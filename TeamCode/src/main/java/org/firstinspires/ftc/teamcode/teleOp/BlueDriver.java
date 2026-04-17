package org.firstinspires.ftc.teamcode.teleOp;

import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

@TeleOp(name="BlueDriver", group="Linear Opmode")
public class BlueDriver extends LinearOpMode {

    ShooterSubsystem shooter;
    IntakeSubsystem intake;
    Follower follower;

    DcMotor leftFront, rightFront, leftBack, rightBack;

    // Added WAIT_FOR_GATE to the state machine
    enum ShotState { IDLE, WAIT_FOR_GATE, RUN_INTAKE, CLOSE_GATE }
    ShotState currentShotState = ShotState.IDLE;
    ElapsedTime manualOnTimer = new ElapsedTime();
    ElapsedTime shotTimer = new ElapsedTime();

    ElapsedTime breakTimer = new ElapsedTime();
    boolean beamWasBroken = false;
    boolean intakeAutoStopped = false;

    // Toggle variables for Right Trigger
    boolean intakeDisabledManually = false;
    boolean lastRT = false;

    private Pose startPose = new Pose(7.7, 8.1, Math.toRadians(0));
    private boolean lastLB = false;
    private boolean lastRB = false;

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);

        leftFront = hardwareMap.get(DcMotor.class, "LF");
        rightFront = hardwareMap.get(DcMotor.class, "RF");
        leftBack = hardwareMap.get(DcMotor.class, "LB");
        rightBack = hardwareMap.get(DcMotor.class, "RB");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter.closeGate();
        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            Pose currentPose = follower.getPose();

            // Get the current velocity vector from Pedro Pathing for SOTM
            Vector currentVelocity = follower.getVelocity();

            // 1. DRIVING
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            leftFront.setPower((y + x + rx) / denominator);
            leftBack.setPower((y - x + rx) / denominator);
            rightFront.setPower((y - x - rx) / denominator);
            rightBack.setPower((y + x - rx) / denominator);

            // 2. SHOOTER & TRIM
            if (gamepad1.a) { shooter.trackingActive = true; shooter.enableFlywheels(); }
            if (gamepad1.b) { shooter.trackingActive = false; shooter.disableFlywheels(); }

            if (gamepad1.left_bumper && !lastLB)  shooter.trimDegrees -= 1.0;
            if (gamepad1.right_bumper && !lastRB) shooter.trimDegrees += 1.0;
            lastLB = gamepad1.left_bumper;
            lastRB = gamepad1.right_bumper;

            // Updated alignTurret call injecting X and Y velocities
            shooter.alignTurret(
                    currentPose.getX(),
                    currentPose.getY(),
                    currentPose.getHeading(),
                    currentVelocity.getXComponent(),
                    currentVelocity.getYComponent(),
                    true,
                    telemetry,
                    0,
                    false
            );

// 3. INTAKE LOGIC (MANUAL TOGGLE ON RIGHT TRIGGER)
            boolean rtPressed = gamepad1.right_trigger > 0.5;
            if (rtPressed && !lastRT) {
                intakeDisabledManually = !intakeDisabledManually;

                if (intakeDisabledManually) {
                    intake.intakeOff();
                    shooter.openGate();
                } else {
                    shooter.closeGate();
                    manualOnTimer.reset();
                }
            }
            lastRT = rtPressed;

            boolean isCurrentlyBroken = intake.isFull();

            if (currentShotState == ShotState.IDLE) {
                if (intakeDisabledManually) {
                    intake.intakeOff(); // Gate is already open from the toggle logic above
                }
                else if (manualOnTimer.seconds() < 0.5) {
                    intake.intakeOff(); // Wait for gate to close
                }
                else if (isCurrentlyBroken) {
                    // Auto-Stop Sequence
                    if (!beamWasBroken) {
                        breakTimer.reset();
                        beamWasBroken = true;
                    }
                    if (breakTimer.seconds() >= 0.3 && !intakeAutoStopped) {
                        intake.intakeOff();
                        intakeAutoStopped = true;
                    }
                    if (breakTimer.seconds() >= 0.4 && intakeAutoStopped) {
                        shooter.openGate();
                    }
                }
                else {
                    intake.intakeFull();
                    beamWasBroken = false;
                    intakeAutoStopped = false;
                }
            }

            // 4. SHOT SEQUENCE (ON LEFT TRIGGER)
            if (gamepad1.left_trigger > 0.5 && currentShotState == ShotState.IDLE) {
                currentShotState = ShotState.WAIT_FOR_GATE;
                shotTimer.reset();
            }

            switch (currentShotState) {
                case WAIT_FOR_GATE:
                    if (shotTimer.seconds() >= 0.05) {
                        intake.intakeCustom();
                        shotTimer.reset();
                        currentShotState = ShotState.RUN_INTAKE;
                    }
                    break;
                case RUN_INTAKE:
                    if (shotTimer.seconds() >= 0.4) {
                        intake.intakeOff();
                        shotTimer.reset();
                        currentShotState = ShotState.CLOSE_GATE;
                    }
                    break;
                case CLOSE_GATE:
                    if (shotTimer.seconds() >= 0.3) {
                        shooter.closeGate();
                        currentShotState = ShotState.IDLE;
                        intakeAutoStopped = false;
                        beamWasBroken = false;
                    }
                    break;
            }

            telemetry.addData("Intake System", intakeDisabledManually ? "DISABLED" : "AUTO");
            telemetry.addData("Shot Phase", currentShotState);
            telemetry.update();
        }
    }
}