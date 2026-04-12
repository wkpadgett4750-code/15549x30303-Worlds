package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Turret Tracking Test", group = "Test")
public class TestTurretTeleOp extends OpMode {

    private Follower follower;
    private ShooterSubsystem shooter;

    // Drive motors
    private DcMotorEx leftFront, leftBack, rightFront, rightBack;

    // Driver trim (degrees)
    private double driverTrim = 0;

    // Button edge tracking
    private boolean lastLB = false, lastRB = false;

    // Starting pose (your request)
    private Pose startPose = new Pose(
            7.70519724409,
            8.12401575,
            Math.toRadians(0)
    );

    // Safe pose updated every loop
    private Pose safePose = startPose;

    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);
        shooter  = new ShooterSubsystem(hardwareMap);

        // Drive motors
        leftFront  = hardwareMap.get(DcMotorEx.class, "LF");
        leftBack   = hardwareMap.get(DcMotorEx.class, "LB");
        rightFront = hardwareMap.get(DcMotorEx.class, "RF");
        rightBack  = hardwareMap.get(DcMotorEx.class, "RB");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set starting pose
        follower.setStartingPose(startPose);

        telemetry.addLine("Turret Tracking Test READY");
        telemetry.update();
    }

    @Override
    public void loop() {

        // -----------------------------
        // UPDATE PEDRO ODOMETRY
        // -----------------------------
        follower.update();
        Pose cp = follower.getPose();
        if (cp != null) safePose = cp;

        // -----------------------------
        // DRIVE CONTROL (Arcade)
        // -----------------------------
        double y  = -gamepad1.right_stick_y;
        double x  = gamepad1.right_stick_x * 1.1;
        double rx = gamepad1.left_stick_x;

        double den = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        leftFront.setPower((y + x + rx) / den);
        leftBack.setPower((y - x + rx) / den);
        rightFront.setPower((y - x - rx) / den);
        rightBack.setPower((y + x - rx) / den);

        // -----------------------------
        // DRIVER TRIM CONTROL
        // -----------------------------
        if (gamepad1.right_bumper && !lastRB) driverTrim += 1;
        if (gamepad1.left_bumper  && !lastLB) driverTrim -= 1;

        lastRB = gamepad1.right_bumper;
        lastLB = gamepad1.left_bumper;

        // -----------------------------
        // TURRET TRACKING EVERY LOOP
        // -----------------------------
        shooter.alignTurret(
                safePose.getX(),
                safePose.getY(),
                safePose.getHeading(),
                true,          // aiming at blue goal
                telemetry,
                0, 0,          // magVel, thetaVel (unused)
                driverTrim,    // driver trim offset
                false          // isAuto = false
        );
        shooter.disableFlywheels();
        shooter.closeGate();
        // -----------------------------
        // TELEMETRY
        // -----------------------------
        telemetry.addLine("=== Pose ===");
        telemetry.addData("X", safePose.getX());
        telemetry.addData("Y", safePose.getY());
        telemetry.addData("H (deg)", Math.toDegrees(safePose.getHeading()));

        telemetry.addLine("=== Turret ===");
        telemetry.addData("Ticks", shooter.getTurretPos());
        telemetry.addData("Driver Trim", driverTrim);

        telemetry.update();
    }
}