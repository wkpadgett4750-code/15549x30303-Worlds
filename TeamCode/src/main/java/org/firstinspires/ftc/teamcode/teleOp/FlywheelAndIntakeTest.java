package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

@TeleOp(name = "FlywheelAndIntakeTest")
public class FlywheelAndIntakeTest extends LinearOpMode {

    ShooterSubsystem shooter;

    DcMotorEx intakeLeft, intakeRight;

    @Override
    public void runOpMode() {

        shooter = new ShooterSubsystem(hardwareMap);

        intakeLeft = hardwareMap.get(DcMotorEx.class, "intake1");
        intakeRight = hardwareMap.get(DcMotorEx.class, "intake2");

        // Intake motors run opposite directions
        intakeLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Make intake run as fast as possible using encoders
        intakeLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // -----------------------------
            // FLYWHEEL CONTROL (Triggers)
            // -----------------------------
            if (gamepad1.left_trigger > 0.1) {
                shooter.setFlywheelVelocity(1500, true);   // ENCODER velocity mode
            }

            if (gamepad1.right_trigger > 0.1) {
                shooter.setFlywheelVelocity(0, true);      // stop
            }


            // -----------------------------
            // INTAKE CONTROL (Bumpers)
            // -----------------------------
            if (gamepad1.left_bumper) {
                // Full speed using encoder mode
                intakeLeft.setVelocity(1500);   // ticks/sec equivalent handled internally
                intakeRight.setVelocity(1500);
            }

            if (gamepad1.right_bumper) {
                intakeLeft.setPower(0);
                intakeRight.setPower(0);
            }


            // -----------------------------
            // TELEMETRY
            // -----------------------------
            //telemetry.addData("Flywheel Target (RPM)", shooter.getTargetVelocity());
           // telemetry.addData("Flywheel Current (RPM)", shooter.getCurrentVelocity());
            telemetry.addData("Intake", gamepad1.left_bumper ? "ON" : "OFF");
            telemetry.update();
        }
    }
}