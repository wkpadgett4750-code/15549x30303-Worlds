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

@Autonomous(name = "BottomBlue21", group = "Autonomous")
@Configurable
public class BottomBlue21 extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;

    // Subsystems
    ShooterSubsystem shooter;
    IntakeSubsystem intake;

    private int pathState = 0;
    private Paths paths;
    private int gateCycles = 0;

    // --- NEW DEGREE-BASED TURRET TARGET ---
    // Set this to 90 or -90 to match your manual buttons
    public static double turretTargetDegrees = -95.0;

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
        follower.setStartingPose(new Pose(18.1, 124.5, Math.toRadians(-36)));
        PoseStorage.lastRoute = PoseStorage.AutoRoute.CLOSE_BLUE;

        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);

        shooter.closeGate();
        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
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

        // 1. CALCULATE DISTANCE
        double distToGoal = Math.hypot(ShooterSubsystem.blueGoalX - cp.getX(), ShooterSubsystem.blueGoalY - cp.getY());

        // 2. UPDATED TURRET LOGIC
        // We use the same setTurretPosition call from your TeleOp.
        // This keeps it at a fixed 90-degree offset throughout the auto.
        shooter.setTurretPosition(turretTargetDegrees);

        // 3. CONTINUOUS SHOOTER ADJUSTMENT
        shooter.setFlywheelVelocity(shooter.interpolate(distToGoal, ShooterSubsystem.distances, ShooterSubsystem.rpms), true);
        shooter.setHoodPosition(shooter.interpolate(distToGoal, ShooterSubsystem.distances, ShooterSubsystem.hoods));

        // 4. BACKGROUND AUTO-INTAKE
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

        // 5. SHOT SEQUENCE STATE MACHINE
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

        // 6. PATHING STATE MACHINE
        autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("Turret Angle", turretTargetDegrees);
        panelsTelemetry.debug("Shot State", currentShotState);
        panelsTelemetry.update(telemetry);
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
                pathState = 1;
                break;
            case 1:
                if (!follower.isBusy()) { spinUpTimer.reset(); pathState = 15; }
                break;
            case 15:
                if (spinUpTimer.seconds() >= .05) { triggerShot(); pathState = 2; }
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
                if (spinUpTimer.seconds() >= .9) { follower.followPath(paths.GateShot); pathState = 7; }
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
                    // You can even update the turret target mid-auto!
                    turretTargetDegrees = -70.0;
                    follower.followPath(paths.ShootSpike1);
                    pathState = 10;
                }
                break;
            case 10:
                if (!follower.isBusy()) { triggerShot(); pathState = 11; }
                break;
            case 11:
                if (currentShotState == ShotState.IDLE) {
                    PoseStorage.currentPose = follower.getPose();
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
                                    new Pose(18.100, 119.500),
                                    new Pose(41.154, 101.158),
                                    new Pose(55.850, 85.907)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-36), Math.toRadians(-135))
                    .build();

            IntakeSpike2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(55.850, 85.907),
                                    new Pose(51.079, 58.463),
                                    new Pose(12.065, 63.056)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-135), Math.toRadians(-180))
                    .build();

            ShootSpike2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(12.065, 63.056),
                                    new Pose(50.290, 68.664),
                                    new Pose(56.290, 86.178)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-135))
                    .build();

            GateTake = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(56.290, 86.178),
                                    new Pose(46.495, 75.005),
                                    new Pose(8.51, 63.139)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-135), Math.toRadians(-215))
                    .build();

            GateShot = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(8.664, 59.439),
                                    new Pose(39.500, 62.374),
                                    new Pose(55.981, 86.131)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-210), Math.toRadians(-135))
                    .build();

            IntakeSpike1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(55.981, 86.131),
                                    new Pose(15.252, 85.458)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-155), Math.toRadians(-180))
                    .build();

            ShootSpike1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(15.252, 85.458),
                                    new Pose(49.748, 115.673)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-90))
                    .build();

        }

    }
    @Override
    public void stop() {
        // This is the "Safety Net" that saves the pose no matter what
        PoseStorage.currentPose = follower.getPose();
    }
}
