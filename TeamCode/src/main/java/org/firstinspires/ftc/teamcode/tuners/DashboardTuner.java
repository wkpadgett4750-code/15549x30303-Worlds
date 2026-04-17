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

@TeleOp(name="FullPanels: Manual Shooter Tuner")
public class DashboardTuner extends LinearOpMode {

    ShooterSubsystem shooter;
    IntakeSubsystem intake;
    Follower follower;

    DcMotor leftFront, rightFront, leftBack, rightBack;

    public static boolean RUN_TRACKING = false;
    public static boolean RUN_FLYWHEELS = false;
    public static boolean RUN_INTAKE = false;

    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;

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

            // 1. DRIVING
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;
            double den = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            leftFront.setPower((y + x + rx) / den);
            leftBack.setPower((y - x + rx) / den);
            rightFront.setPower((y - x - rx) / den);
            rightBack.setPower((y + x - rx) / den);

            // 2. TOGGLES
            if (gamepad1.a && !lastA) RUN_TRACKING = !RUN_TRACKING;
            lastA = gamepad1.a;

            if (gamepad1.b && !lastB) RUN_INTAKE = !RUN_INTAKE;
            lastB = gamepad1.b;

            if (gamepad1.x && !lastX) RUN_FLYWHEELS = !RUN_FLYWHEELS;
            lastX = gamepad1.x;

            // 3. TURRET AIMING (Bypassing Internal Shooter Logic)
            if (RUN_TRACKING) {
                shooter.trackingActive = true;
                // We set 'flywheelsEnabled' to FALSE inside the subsystem briefly
                // so alignTurret doesn't try to interpolate the hood.
                shooter.flywheelsEnabled = false;
                shooter.alignTurret(cp.getX(), cp.getY(), cp.getHeading(), 0, 0, true, null, 0, false);
            } else {
                shooter.stopTurret();
            }

            // 4. MANUAL SHOOTER TUNING (Using Panel Values)
            if (RUN_FLYWHEELS) {
                // This ignores the distance math and uses your sliders directly
                shooter.setFlywheelVelocity(ShooterSubsystem.tuningRPM, false);
                shooter.setHoodPosition(ShooterSubsystem.tuningHoodPos);
            } else {
                shooter.disableFlywheels();
            }

            // 5. INTAKE
            if (RUN_INTAKE) intake.intakeFull();
            else intake.intakeOff();

            // 6. TELEMETRY
            telemetry.addData("--- TUNING MODE ---", "MANUAL");
            telemetry.addData("Dist to Goal", Math.hypot(ShooterSubsystem.blueGoalX - cp.getX(), ShooterSubsystem.blueGoalY - cp.getY()));
            telemetry.addData("Target RPM", ShooterSubsystem.tuningRPM);
            telemetry.addData("Actual RPM", shooter.getCurrentVelocity());
            telemetry.addData("Hood Pos", ShooterSubsystem.tuningHoodPos);
            telemetry.addData("Flywheel kP", ShooterSubsystem.kP);
            telemetry.addData("Flywheel kF", ShooterSubsystem.kF);
            telemetry.update();
        }
    }
}