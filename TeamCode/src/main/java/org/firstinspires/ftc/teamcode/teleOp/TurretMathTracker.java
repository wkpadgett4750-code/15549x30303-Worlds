package org.firstinspires.ftc.teamcode.teleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp(name = "Turret Full: Trim + Drive", group = "Active")
public class TurretMathTracker extends LinearOpMode {

    private Follower follower;
    private CRServo turretLeft, turretRight;
    private DcMotorEx lf, lb, rf, rb;

    // --- TUNED CONSTANTS ---
    public static double blueGoalX = 0.0;
    public static double blueGoalY = 144.0;
    public static double angleOffset = -90;
    public static double tSlope = -67.9055;
    public static double turretOffsetX = 1.5;
    public static double turretOffsetY = 0.0;
    public static int turretMin = -8500;
    public static int turretMax = 13000;

    public static double tp = 0.00021, ti = 0, td = 0.00001, ts = 0.0731;
    public static int deadband = 10;
    public static double ticksPerRev = Math.abs(tSlope * 360.0);

    private Pose startPose = new Pose(7.70519724409, 8.12401575, Math.toRadians(0));
    private double integralSum = 0;
    private double lastError = 0;

    // Toggle & Trim Logic
    private boolean trackingActive = false;
    private boolean lastYState = false;
    private boolean lastLBState = false;
    private boolean lastRBState = false;
    public static double trimDegrees = 0.0;

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        lf = hardwareMap.get(DcMotorEx.class, "LF");
        lb = hardwareMap.get(DcMotorEx.class, "LB");
        rf = hardwareMap.get(DcMotorEx.class, "RF");
        rb = hardwareMap.get(DcMotorEx.class, "RB");

        lf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);

        turretLeft = hardwareMap.get(CRServo.class, "turret1");
        turretRight = hardwareMap.get(CRServo.class, "turret2");
        turretLeft.setDirection(CRServo.Direction.REVERSE);
        turretRight.setDirection(CRServo.Direction.REVERSE);

        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            Pose currentPose = follower.getPose();
            double heading = currentPose.getHeading();

            // --- 1. DRIVE ---
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double turn = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(turn), 1);
            lf.setPower((y + x + turn) / denominator);
            lb.setPower((y - x + turn) / denominator);
            rf.setPower((y - x - turn) / denominator);
            rb.setPower((y + x - turn) / denominator);

            // --- 2. BUTTON LOGIC (Toggles & Trim) ---

            // Toggle Tracking (Y Button)
            if (gamepad1.y && !lastYState) trackingActive = !trackingActive;
            lastYState = gamepad1.y;

            // Trim Left (Left Bumper) - Decrements because of your tSlope orientation
            if (gamepad1.left_bumper && !lastLBState) trimDegrees -= 1.0;
            lastLBState = gamepad1.left_bumper;

            // Trim Right (Right Bumper)
            if (gamepad1.right_bumper && !lastRBState) trimDegrees += 1.0;
            lastRBState = gamepad1.right_bumper;

            // Reset Trim (Left Stick Button)
            if (gamepad1.left_stick_button) trimDegrees = 0;

            // --- 3. TURRET MATH ---
            double cos = Math.cos(heading);
            double sin = Math.sin(heading);
            double adjX = currentPose.getX() + (turretOffsetX * cos - turretOffsetY * sin);
            double adjY = currentPose.getY() + (turretOffsetX * sin + turretOffsetY * cos);

            double angleToGoal = Math.toDegrees(Math.atan2(blueGoalX - adjX, blueGoalY - adjY));

            // Added trimDegrees here
            double targetRel = angleToGoal + Math.toDegrees(heading) - angleOffset + trimDegrees;

            while (targetRel > 180)  targetRel -= 360;
            while (targetRel < -180) targetRel += 360;

            int targetTicks = (int) (targetRel * tSlope);

            // Continuous Wrap
            if (targetTicks < turretMin) {
                int alt = (int) (targetTicks + (int)ticksPerRev);
                if (alt <= turretMax) targetTicks = alt;
            } else if (targetTicks > turretMax) {
                int alt = (int) (targetTicks - (int)ticksPerRev);
                if (alt >= turretMin) targetTicks = alt;
            }

            // --- 4. PIDF ---
            int currentTicks = rf.getCurrentPosition();
            double error = targetTicks - currentTicks;
            double turretPower = 0;

            if (trackingActive) {
                if (Math.abs(error) > deadband) {
                    integralSum += error;
                    double derivative = error - lastError;
                    turretPower = (error * tp) + (integralSum * ti) + (derivative * td) + Math.copySign(ts, error);
                    lastError = error;
                } else {
                    turretPower = 0;
                    integralSum = 0;
                }

                double finalTurretPower = Range.clip(turretPower, -0.9, 0.9);
                turretLeft.setPower(finalTurretPower);
                turretRight.setPower(finalTurretPower);
            } else {
                turretLeft.setPower(0);
                turretRight.setPower(0);
                integralSum = 0;
            }

            // --- TELEMETRY ---
            telemetry.addData("Status", trackingActive ? "TRACKING (Y to Stop)" : "OFF (Y to Start)");
            telemetry.addData("Trim", "%.1f°", trimDegrees);
            telemetry.addData("Err/Tgt", "%d / %d", (int)error, targetTicks);
            telemetry.update();
        }
    }
}