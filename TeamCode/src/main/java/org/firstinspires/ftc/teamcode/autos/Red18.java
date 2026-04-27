package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

@Autonomous(name = "Red18", group = "Autonomous")
@Configurable
public class Red18 extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;

    // Subsystems
    ShooterSubsystem shooter;
    IntakeSubsystem intake;

    private int pathState = 0;
    private Paths paths;
    private int gateCycles = 0;

    // --- TURRET TARGET (Adjusted for Red Side) ---
    public static double turretTargetDegrees = 90.0;

    // --- STATE TIMERS & FLAGS ---
    enum ShotState { IDLE, OPEN_GATE, RUN_INTAKE, CLOSE_GATE, COOLDOWN }
    ShotState currentShotState = ShotState.IDLE;

    ElapsedTime shotTimer = new ElapsedTime();
    ElapsedTime spinUpTimer = new ElapsedTime();
    ElapsedTime breakTimer = new ElapsedTime();
    boolean beamWasBroken = false;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        // Start Pose matches the beginning of PreLoadShot
        follower.setStartingPose(new Pose(125.9, 119.5, Math.toRadians(216)));

        // Handshake for TeleOp failsafes


        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);

        shooter.closeGate();
        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Red18 Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        shooter.enableFlywheels();
    }

    @Override
    public void loop() {
        follower.update();
        Pose cp = follower.getPose();
        PoseStorage.currentPose = follower.getPose();


        // 1. CALCULATE DISTANCE (Using Red Goal)
        double distToGoal = Math.hypot(ShooterSubsystem.redGoalX - cp.getX(), ShooterSubsystem.redGoalY - cp.getY());

        // 2. TURRET CONTROL
        shooter.setTurretPosition(turretTargetDegrees);

        // 3. SHOOTER INTERPOLATION
        shooter.setFlywheelVelocity(shooter.interpolate(distToGoal, ShooterSubsystem.distances, ShooterSubsystem.rpms), true);
        shooter.setHoodPosition(shooter.interpolate(distToGoal, ShooterSubsystem.distances, ShooterSubsystem.hoods));

        // 4. BACKGROUND INTAKE
        boolean isCurrentlyBroken = intake.isFull();
        if (currentShotState == ShotState.IDLE) {
            if (isCurrentlyBroken) {
                if (!beamWasBroken) { breakTimer.reset(); beamWasBroken = true; }
                if (breakTimer.seconds() >= 0.3) intake.intakeOff();
            } else {
                intake.intakeFull();
                beamWasBroken = false;
                shooter.closeGate();
            }
        }

        // 5. SHOT SEQUENCE
        updateShotSequence(distToGoal);

        // 6. PATHING
        autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", cp.getX());
        panelsTelemetry.debug("Y", cp.getY());
        panelsTelemetry.update(telemetry);
    }

    private void updateShotSequence(double distToGoal) {
        switch (currentShotState) {
            case OPEN_GATE:
                if (shotTimer.seconds() >= 0.05) {
                    if (distToGoal > 140) intake.farShot(); else intake.autoShot();
                    shotTimer.reset();
                    currentShotState = ShotState.RUN_INTAKE;
                }
                break;
            case RUN_INTAKE:
                if (shotTimer.seconds() >= 0.43) {
                    intake.intakeOff();
                    shooter.closeGate();
                    shotTimer.reset();
                    currentShotState = ShotState.COOLDOWN;
                }
                break;
            case COOLDOWN:
                if (shotTimer.seconds() >= 0.05) {
                    currentShotState = ShotState.IDLE;
                    beamWasBroken = false;
                }
                break;
        }
    }

    private void triggerShot() {
        if (currentShotState == ShotState.IDLE) {
            currentShotState = ShotState.OPEN_GATE;
            shooter.openGate();
            shotTimer.reset();
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(paths.PreLoadShot);
                pathState = 15;
                break;
            case 15:
                if (!follower.isBusy()) { triggerShot(); pathState = 2; }
                break;
            case 2:
                if (currentShotState == ShotState.IDLE) {
                    follower.followPath(paths.IntakeSpike2);
                    pathState = 3;
                }
                break;
            case 3:
                if (!follower.isBusy()) { follower.followPath(paths.ShootSpike2); pathState = 4; }
                break;
            case 4:
                if (!follower.isBusy()) { triggerShot(); pathState = 5; }
                break;
            case 5:
                if (currentShotState == ShotState.IDLE) { follower.followPath(paths.GateTake); pathState = 6; }
                break;
            case 6:
                if (!follower.isBusy()) { spinUpTimer.reset(); pathState = 16; }
                break;
            case 16:
                if (spinUpTimer.seconds() >= 0.65) { follower.followPath(paths.GateShot); pathState = 7; }
                break;
            case 7:
                if (!follower.isBusy()) { triggerShot(); pathState = 8; }
                break;
            case 8:
                if (currentShotState == ShotState.IDLE) {
                    gateCycles++;
                    if (gateCycles < 3) { follower.followPath(paths.GateTake); pathState = 6; }
                    else { follower.followPath(paths.IntakeSpike1); pathState = 9; }
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    turretTargetDegrees = 70.0;
                    follower.followPath(paths.ShootSpike1);
                    pathState = 10;
                }
                break;
            case 10:
                if (!follower.isBusy()) { triggerShot(); pathState = 11; }
                break;
            case 11:
                if (currentShotState == ShotState.IDLE) {
                    pathState = -1;
                }
                break;
        }
    }

    public static class Paths {
        public PathChain PreLoadShot, IntakeSpike2, ShootSpike2, GateTake, GateShot, IntakeSpike1, ShootSpike1;
        public Paths(Follower follower) {
            PreLoadShot = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(125.900, 119.500),
                                    new Pose(102.846, 101.158),
                                    new Pose(88.150, 85.907)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(216), Math.toRadians(315))
                    .build();

            IntakeSpike2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(88.150, 85.907),
                                    new Pose(92.921, 58.463),
                                    new Pose(131.935, 63.056)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(360))
                    .build();

            ShootSpike2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(131.935, 63.056),
                                    new Pose(93.710, 68.664),
                                    new Pose(87.710, 86.178)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(360), Math.toRadians(315))
                    .build();

            GateTake = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(87.710, 86.178),
                                    new Pose(101.505, 69.005),
                                    new Pose(135.336, 61.439)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(390))
                    .build();

            GateShot = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(135.336, 59.439),
                                    new Pose(104.500, 62.374),
                                    new Pose(88.019, 86.131)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(390), Math.toRadians(315))
                    .build();

            IntakeSpike1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(88.019, 86.131),
                                    new Pose(128.748, 85.458)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(335), Math.toRadians(360))
                    .build();

            ShootSpike1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(128.748, 85.458),
                                    new Pose(94.252, 115.673)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(360), Math.toRadians(270))
                    .build();
        }
    }

    @Override
    public void stop() {
        // Save the final pose for RedDriver
        PoseStorage.currentPose = follower.getPose();
    }
}