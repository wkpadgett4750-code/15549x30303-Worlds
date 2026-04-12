package org.firstinspires.ftc.teamcode;

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
@TeleOp(name = "Turret Full: Drive + Toggle Tracking", group = "Active")
public class TurretMathTracker extends LinearOpMode {

    private Follower follower;
    private DcMotorEx turretEncoder;
    private CRServo turretLeft, turretRight;
    private DcMotorEx leftFront, leftRear, rightFront, rightRear;

    // --- TUNED CONSTANTS ---
    public static double blueGoalX = 0.0;
    public static double blueGoalY = 144.0;
    public static double angleOffset = -90;
    public static double tSlope = -67.9055;
    public static double turretOffsetX = 1.5;
    public static double turretOffsetY = 0.0;
    public static int turretMin = -8500;
    public static int turretMax = 13000;

    // --- YOUR TUNED PIDF + DEADBAND ---
    public static double tp = 0.00021, ti = 0, td = 0.00001, ts = 0.0731;
    public static int deadband = 10;
    public static double ticksPerRev = Math.abs(tSlope * 360.0);

    private Pose startPose = new Pose(7.70519724409, 8.12401575, Math.toRadians(0));
    private double integralSum = 0;
    private double lastError = 0;

    // Toggle logic
    private boolean trackingActive = false;
    private boolean lastBumperState = false;

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // Drive Motors
        leftFront = hardwareMap.get(DcMotorEx.class, "LF");
        leftRear = hardwareMap.get(DcMotorEx.class, "LB");
        rightFront = hardwareMap.get(DcMotorEx.class, "RF");
        rightRear = hardwareMap.get(DcMotorEx.class, "RB");

        // Reverse left side motors so positive power moves forward
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        // Turret Hardware
        turretLeft = hardwareMap.get(CRServo.class, "turret1");
        turretRight = hardwareMap.get(CRServo.class, "turret2");
        turretLeft.setDirection(CRServo.Direction.REVERSE);
        turretRight.setDirection(CRServo.Direction.REVERSE);

        turretEncoder = hardwareMap.get(DcMotorEx.class, "RF");
        turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            Pose currentPose = follower.getPose();
            double heading = currentPose.getHeading();

            // --- 1. MECANUM DRIVE (Robot Centric) ---
            double y = -gamepad1.left_stick_y; // Forward/Back
            double x = gamepad1.left_stick_x * 1.1; // Strafe (counteract friction)
            double rx = gamepad1.right_stick_x; // Turn

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            leftFront.setPower((y + x + rx) / denominator);
            leftRear.setPower((y - x + rx) / denominator);
            rightFront.setPower((y - x - rx) / denominator);
            rightRear.setPower((y + x - rx) / denominator);

            // --- 2. TRACKING TOGGLE LOGIC ---
            if (gamepad1.right_bumper && !lastBumperState) {
                trackingActive = !trackingActive;
            }
            lastBumperState = gamepad1.right_bumper;

            // --- 3. TURRET MATH ---
            double cos = Math.cos(heading);
            double sin = Math.sin(heading);
            double adjX = currentPose.getX() + (turretOffsetX * cos - turretOffsetY * sin);
            double adjY = currentPose.getY() + (turretOffsetX * sin + turretOffsetY * cos);

            double angleToGoal = Math.toDegrees(Math.atan2(blueGoalX - adjX, blueGoalY - adjY));
            double targetRel = angleToGoal + Math.toDegrees(heading) - angleOffset;

            while (targetRel > 180)  targetRel -= 360;
            while (targetRel < -180) targetRel += 360;

            int targetTicks = (int) (targetRel * tSlope);

            // Continuous Wrap
            if (targetTicks < turretMin) {
                int alt = (int) (targetTicks + ticksPerRev);
                if (alt <= turretMax) targetTicks = alt;
            } else if (targetTicks > turretMax) {
                int alt = (int) (targetTicks - ticksPerRev);
                if (alt >= turretMin) targetTicks = alt;
            }

            // --- 4. PIDF CONTROL + DEADBAND ---
            int currentTicks = turretEncoder.getCurrentPosition();
            double error = targetTicks - currentTicks;
            double power = 0;

            if (trackingActive) {
                if (Math.abs(error) > deadband) {
                    integralSum += error;
                    double derivative = error - lastError;
                    power = (error * tp) + (integralSum * ti) + (derivative * td);
                    power += Math.copySign(ts, error);
                    lastError = error;
                } else {
                    power = 0;
                    integralSum = 0; // Reset I-term when in deadband
                }

                double finalPower = Range.clip(power, -0.9, 0.9);
                turretLeft.setPower(finalPower);
                turretRight.setPower(finalPower);
            } else {
                turretLeft.setPower(0);
                turretRight.setPower(0);
                integralSum = 0;
            }

            // --- TELEMETRY ---
            telemetry.addData("Tracking", trackingActive ? "ENABLED" : "DISABLED (Press RB)");
            telemetry.addData("Target", targetTicks);
            telemetry.addData("Actual", currentTicks);
            telemetry.addData("Error", error);
            telemetry.update();
        }
    }
}