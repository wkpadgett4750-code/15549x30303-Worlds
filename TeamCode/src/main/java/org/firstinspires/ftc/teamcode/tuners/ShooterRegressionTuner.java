package org.firstinspires.ftc.teamcode.tuners;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

@Disabled
@Configurable
@TeleOp(name = "Shooter Regression Tuner", group = "TeleOp")
public class ShooterRegressionTuner extends OpMode {
    // Subsystems
    ShooterSubsystem shooter;
    IntakeSubsystem intake;
    Follower follower;

    // Drive Motors (Raw control to prevent Pedro crashes)
    private DcMotorEx leftFront, leftBack, rightFront, rightBack;

    // Logic States
    enum IntakeState { IDLE, INTAKING, Full }
    private IntakeState currentIntakeState = IntakeState.IDLE;

    private boolean lastX = false;
    private boolean lastLB = false;
    private boolean lastRB = false;

    @Override
    public void init() {
        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        follower = Constants.createFollower(hardwareMap);

        // Start center of bot at 8,8 (tucked in corner)
        follower.setStartingPose(new Pose(7.70519724409, 8.12401575, 0));

        // Hardware for raw motor driving
        leftFront = hardwareMap.get(DcMotorEx.class, "LF");
        leftBack = hardwareMap.get(DcMotorEx.class, "LB");
        rightFront = hardwareMap.get(DcMotorEx.class, "RF");
        rightBack = hardwareMap.get(DcMotorEx.class, "RB");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        shooter.disableFlywheels();
        shooter.closeGate();
        intake.intakeOff();
    }

    @Override
    public void loop() {
        follower.update();
        Pose cp = follower.getPose();

        // --- 1. ROBOT CENTRIC DRIVE ---
        // Translation: Left Stick | Rotation: Right Stick
        double y = -gamepad1.right_stick_x;
        double x = gamepad1.right_stick_y * 1.1;
        double rx = gamepad1.left_stick_y;

        double den = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        leftFront.setPower((y + x + rx) / den);
        leftBack.setPower((y - x + rx) / den);
        rightFront.setPower((y - x - rx) / den);
        rightBack.setPower((y + x - rx) / den);

        // --- 2. SHOOTER TUNING INPUTS ---

        // Adjust Velocity Target (Bumpers)
        if (gamepad1.right_bumper && !lastRB) {
            ShooterSubsystem.tuningRPM += 100;
            shooter.enableFlywheels();
            gamepad1.rumble(100);
        }
        else if (gamepad1.left_bumper && !lastLB) {
            ShooterSubsystem.tuningRPM -= 100;
            shooter.enableFlywheels();
            gamepad1.rumble(100);
        }

        // Adjust Hood Position (D-Pad)
        if (gamepad1.dpad_up) ShooterSubsystem.tuningHoodPos += 0.001;
        if (gamepad1.dpad_down) ShooterSubsystem.tuningHoodPos -= 0.001;

        // Ensure values stay within the safety limits set in your Subsystem
        ShooterSubsystem.tuningHoodPos = Range.clip(ShooterSubsystem.tuningHoodPos,
                ShooterSubsystem.hoodMinPos,
                ShooterSubsystem.hoodMaxPos);

        // --- 3. INTAKE & FEED LOGIC ---

        // Tap X: Toggle Intake Search (Stops automatically on Stall)
        if (gamepad1.x && !lastX) {
            currentIntakeState = (currentIntakeState == IntakeState.INTAKING) ? IntakeState.IDLE : IntakeState.INTAKING;
        }

        // Tap B: Stop Everything
        if (gamepad1.b) {
            currentIntakeState = IntakeState.IDLE;
            shooter.disableFlywheels();
        }

        // A Button: MANUAL KICK (Hold to shoot)
        // This overrides intake state to feed the ball
        if (gamepad1.a) {
            shooter.openGate();
            intake.intakeCustom(); // Feed ball
        } else {
            // Only update intake if we aren't kicking
            handleIntakeState();
        }

        lastRB = gamepad1.right_bumper; lastLB = gamepad1.left_bumper; lastX = gamepad1.x;

        // --- 4. HARDWARE UPDATES ---
        // We use the "tuning" variables from Dashboard for manual control
        shooter.setFlywheelVelocity(ShooterSubsystem.tuningRPM, false);
        shooter.setHoodPosition(ShooterSubsystem.tuningHoodPos);

        // Manual Aim: Use the Dashboard slider to aim the turret
        shooter.setTurretPosition(ShooterSubsystem.tuningTurretTicks);

        // --- 5. BASIC TELEMETRY ---
        if (cp != null) {
            // Distance to blue goal (0, 144) as requested
            double dist = Math.hypot(ShooterSubsystem.blueGoalX - cp.getX(),
                    ShooterSubsystem.blueGoalY - cp.getY());

            telemetry.addData("Distance to Blue Goal", "%.2f", dist);
            telemetry.addData("Target RPM", ShooterSubsystem.tuningRPM);
            telemetry.addData("Actual RPM", (int)shooter.getCurrentVelocity());
            telemetry.addData("Hood Target", "%.3f", ShooterSubsystem.tuningHoodPos);
            telemetry.addData("Turret Ticks", shooter.getTurretPos());
            telemetry.addData("Intake State", currentIntakeState);
            telemetry.addData("Ready to Fire", shooter.isAtSpeed());
        } else {
            telemetry.addLine("WAITING FOR LOCALIZER POSE...");
        }
        telemetry.update();
    }

    private void handleIntakeState() {
        switch (currentIntakeState) {
            case IDLE:
                intake.intakeOff();
                break;
            case INTAKING:
                intake.intakeFull();
                break;

        }
    }
}