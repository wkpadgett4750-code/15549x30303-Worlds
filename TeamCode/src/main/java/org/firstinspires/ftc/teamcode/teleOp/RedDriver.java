package org.firstinspires.ftc.teamcode.teleOp;

import static org.firstinspires.ftc.teamcode.subsystems.PoseStorage.AutoRoute.BLUE_RESET;
import static org.firstinspires.ftc.teamcode.subsystems.PoseStorage.AutoRoute.CLOSE_BLUE;
import static org.firstinspires.ftc.teamcode.subsystems.PoseStorage.AutoRoute.CLOSE_RED;
import static org.firstinspires.ftc.teamcode.subsystems.PoseStorage.AutoRoute.RED_RESET;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

@TeleOp(name="RedDriver", group="Linear Opmode")
public class RedDriver extends LinearOpMode {

    // Controller 2 State Tracking
    private boolean autoAlignEnabled = true; // Start in Auto
    private boolean lastX2 = false;          // For the toggle
    private double manualTargetDegrees = 0;  // 0 is centered (0.5)

    ShooterSubsystem shooter;
    IntakeSubsystem intake;
    Follower follower;

    DcMotor leftFront, rightFront, leftBack, rightBack;

    enum ShotState { IDLE, OPEN_GATE, WAIT_FOR_GATE, RUN_INTAKE, CLOSE_GATE, COOLDOWN }
    ShotState currentShotState = ShotState.IDLE;
    ElapsedTime shotTimer = new ElapsedTime();

    ElapsedTime breakTimer = new ElapsedTime();
    boolean beamWasBroken = false;
    boolean intakeAutoStopped = false;

    boolean intakeDisabledManually = false;
    boolean lastRT = false;

    private Pose startPose = new Pose(7.7, 8.1, Math.toRadians(180));
    private boolean lastLB = false;
    private boolean lastRB = false;
    // Define your "Home" position for recalibration (adjust these coordinates!)


    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        // --- THE FAILSAFE LOGIC ---
        if (PoseStorage.currentPose != null) {
            // 1. Success! Load the exact pose from Auto
            follower.setStartingPose(PoseStorage.currentPose);
        } else {
            // 2. Failsafe: Auto didn't run or save.
            // Load the default for this specific TeleOp/Side.
            follower.setStartingPose(PoseStorage.getFailsafePose(CLOSE_RED));
        }

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
            double distToGoal = Math.hypot(ShooterSubsystem.redGoalX - currentPose.getX(), ShooterSubsystem.redGoalY - currentPose.getY());

            // 1. CHASSIS DRIVE (GP1)
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            leftFront.setPower((y + x + rx) / denominator);
            leftBack.setPower((y - x + rx) / denominator);
            rightFront.setPower((y - x - rx) / denominator);
            rightBack.setPower((y + x - rx) / denominator);

            // 2. TURRET MODE TOGGLE (GP2)
            if (gamepad2.x && !lastX2) {
                autoAlignEnabled = !autoAlignEnabled;
                if (!autoAlignEnabled) manualTargetDegrees = 0; // Home on switch
            }
            lastX2 = gamepad2.x;

            // 3. EXECUTION: AUTO VS MANUAL
            if (autoAlignEnabled) {
                // Controller 1 can still kill tracking/flywheels globally
                if (gamepad1.a) { shooter.trackingActive = true; shooter.enableFlywheels(); }
                if (gamepad1.b) { shooter.trackingActive = false; shooter.disableFlywheels(); }

                shooter.alignTurret(currentPose.getX(), currentPose.getY(), currentPose.getHeading(), true, telemetry, 0, false);
            } else {
                // MANUAL SNAPS (GP2)
                if (gamepad2.a) manualTargetDegrees = 90;
                else if (gamepad2.b) manualTargetDegrees = -90;
                else if (gamepad2.y) manualTargetDegrees = 0;

                shooter.setTurretPosition(manualTargetDegrees);

                // 2. RECALIBRATION BUTTON COMBO (GP2)
                // Start + Back is the standard safety combo
                if (gamepad2.start && gamepad2.back) {
                    follower.setPose(PoseStorage.getFailsafePose(RED_RESET));
                    gamepad2.rumble(300);
                }
                // Helpful Telemetry for the drivers
                telemetry.addData("X", currentPose.getX());
                telemetry.addData("Y", currentPose.getY());
                telemetry.addData("Heading", Math.toDegrees(currentPose.getHeading()));
                telemetry.update();
            }

            // 4. TRIM & FLYWHEEL CONTROL (GP1)
            if (gamepad1.left_bumper && !lastLB)  shooter.trimDegrees -= 1.0;
            if (gamepad1.right_bumper && !lastRB) shooter.trimDegrees += 1.0;
            lastLB = gamepad1.left_bumper;
            lastRB = gamepad1.right_bumper;

            // --- 5. INTAKE & SHOT LOGIC (KEEP AS IS) ---
            boolean rtPressed = gamepad1.right_trigger > 0.5;
            if (rtPressed && !lastRT) intakeDisabledManually = !intakeDisabledManually;
            lastRT = rtPressed;

            boolean isCurrentlyBroken = intake.isFull();

            if (currentShotState == ShotState.IDLE) {
                if (intakeDisabledManually) {
                    intake.intakeOff();
                } else if (isCurrentlyBroken) {
                    if (!beamWasBroken) { breakTimer.reset(); beamWasBroken = true; }
                    if (breakTimer.seconds() >= 0.3) { intake.intakeOff(); intakeAutoStopped = true; }
                } else {
                    intake.intakeFull();
                    beamWasBroken = false;
                    intakeAutoStopped = false;
                    shooter.closeGate();
                }
            }

            if (gamepad1.left_trigger > 0.5 && currentShotState == ShotState.IDLE) {
                currentShotState = ShotState.OPEN_GATE;
                shooter.openGate();
                shotTimer.reset();
            }

            switch (currentShotState) {
                case OPEN_GATE:
                    if (shotTimer.seconds() >= 0.15) {
                        if (distToGoal > 140) intake.farShot(); else intake.closeShot();
                        shotTimer.reset();
                        currentShotState = ShotState.RUN_INTAKE;
                    }
                    break;
                case RUN_INTAKE:
                    if (shotTimer.seconds() >= 0.6) {
                        intake.intakeOff();
                        shooter.closeGate();
                        shotTimer.reset();
                        currentShotState = ShotState.COOLDOWN;
                    }
                    break;
                case COOLDOWN:
                    if (shotTimer.seconds() >= 0.1) {
                        currentShotState = ShotState.IDLE;
                        intakeAutoStopped = false;
                        beamWasBroken = false;
                    }
                    break;
            }

            // 6. TELEMETRY
            telemetry.addData("TURRET MODE", autoAlignEnabled ? "AUTO-LOCK" : "MANUAL SNAPS");
            if (!autoAlignEnabled) telemetry.addData("Manual Goal", manualTargetDegrees + "°");
            telemetry.addData("Dist to Goal", distToGoal);
            telemetry.addData("Trim", shooter.trimDegrees + "°");
            telemetry.update();
        }
    }
}
