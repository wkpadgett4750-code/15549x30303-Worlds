package org.firstinspires.ftc.teamcode.tuners;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

@Configurable
@TeleOp(name="DashBoard Tuner")
public class DashboardTuner extends LinearOpMode {

    ShooterSubsystem shooter;
    IntakeSubsystem intake;
    Follower follower;
    DcMotor leftFront, rightFront, leftBack, rightBack;

    public static boolean RUN_TRACKING = false;
    public static boolean USE_DASHBOARD_CTRL = true;
    public static boolean RUN_FLYWHEELS = false;
    public static boolean RUN_INTAKE = false;

    private boolean lastA = false, lastB = false, lastX = false, lastY = false;
    private boolean lastUp = false, lastDown = false;

    @Override
    public void runOpMode() {
        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        follower = Constants.createFollower(hardwareMap);

        leftFront = hardwareMap.get(DcMotor.class, "LF");
        rightFront = hardwareMap.get(DcMotor.class, "RF");
        leftBack = hardwareMap.get(DcMotor.class, "LB");
        rightBack = hardwareMap.get(DcMotor.class, "RB");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            Pose cp = follower.getPose();

            // 1. CHASSIS DRIVE (Half power for easier tuning)
            double driveY = -gamepad1.left_stick_y * 0.5;
            double driveX = gamepad1.left_stick_x * 0.55;
            double rx = gamepad1.right_stick_x * 0.5;
            double den = Math.max(Math.abs(driveY) + Math.abs(driveX) + Math.abs(rx), 1);
            leftFront.setPower((driveY + driveX + rx) / den);
            leftBack.setPower((driveY - driveX + rx) / den);
            rightFront.setPower((driveY - driveX - rx) / den);
            rightBack.setPower((driveY + driveX - rx) / den);

            // 2. TOGGLES
            if (gamepad1.a && !lastA) RUN_TRACKING = !RUN_TRACKING;
            lastA = gamepad1.a;

            if (gamepad1.x && !lastX) RUN_FLYWHEELS = !RUN_FLYWHEELS;
            lastX = gamepad1.x;

            if (gamepad1.y && !lastY) {
                USE_DASHBOARD_CTRL = !USE_DASHBOARD_CTRL;
                shooter.stopTurret(); // Reset the "currentCommandedPos" to current spot
            }
            lastY = gamepad1.y;

            // 3. MANUAL TRIM (D-Pad Up/Down)
            if (gamepad1.dpad_up && !lastUp) shooter.trimDegrees += 0.5;
            lastUp = gamepad1.dpad_up;
            if (gamepad1.dpad_down && !lastDown) shooter.trimDegrees -= 0.5;
            lastDown = gamepad1.dpad_down;

            // 4. TURRET CONTROL
            if (RUN_TRACKING) {
                shooter.trackingActive = true;
                ShooterSubsystem.USE_MANUAL_SERVO = USE_DASHBOARD_CTRL;

                if (USE_DASHBOARD_CTRL) {
                    shooter.setTurretPosition(ShooterSubsystem.tuningTargetDegrees);
                } else {
                    // Full Auto Align
                    shooter.alignTurret(cp.getX(), cp.getY(), cp.getHeading(), true, telemetry, 0, false);
                }
            } else {
                shooter.stopTurret();
            }

            // 5. FLYWHEELS
            if (RUN_FLYWHEELS) {
                shooter.setFlywheelVelocity(ShooterSubsystem.tuningRPM);
                shooter.setHoodPosition(ShooterSubsystem.tuningHoodPos);
            } else {
                shooter.disableFlywheels();
            }

            // 6. TELEMETRY
            telemetry.addLine("--- CONTROLS ---");
            telemetry.addData("A", "Toggle Tracking (" + (RUN_TRACKING ? "ON" : "OFF") + ")");
            telemetry.addData("Y", "Mode: " + (USE_DASHBOARD_CTRL ? "SLIDER" : "AUTO"));
            telemetry.addData("D-Pad", "Trim: " + shooter.trimDegrees + "°");

            telemetry.addLine("\n--- TURRET ---");
            telemetry.addData("Target Deg", USE_DASHBOARD_CTRL ? ShooterSubsystem.tuningTargetDegrees : "AUTO");
            telemetry.addData("Slew Limit", ShooterSubsystem.MAX_SERVO_STEP);

            telemetry.update();
        }
    }
}