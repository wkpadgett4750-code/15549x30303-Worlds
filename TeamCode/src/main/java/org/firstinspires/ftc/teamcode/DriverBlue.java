package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

@TeleOp(name = "DriverBlue", group = "TeleOp")
public class DriverBlue extends LinearOpMode {

    // --- Motors ---
    private DcMotorEx LF, RF, LB, RB;
    private DcMotorEx Shooter, Turret, Intake, Lifter;

    // --- Servos ---
    private Servo IntakeServo, WackerServo, Light;

    // --- Sensors ---
    private IMU imu;
    private DistanceSensor intakeSensor;

    // --- Limelight ---
    private Limelight3A limelight;

    // --- Logic States ---

    private boolean shooterOn = false;
    private boolean lastTrigger = false;
    private boolean highSpeedMode = false;
    private boolean lastLeftBumper = false;
    private double lowShooterVelocity = -1820;
    private double highShooterVelocity = -2400;

    private boolean reversingIntake = false;
    private long reverseStartTime = 0;
    private boolean autoIntakeEnabled = true;
    private boolean objectDetected = false;
    private int cycleCount = 0;
    private int spinnerState = 0;
    private double lastTx = 0;
    private double chaseProgress = 0;

    private final double DEADZONE = 0.05;

    // --- TURRET TUNING FOR 1150 RPM ---
    // Lowered from 0.8 to 0.4 because the motor is much faster
    private double turretPowerScale = 0.85;

    @Override
    public void runOpMode() {
        // --- Hardware Map ---
        LF = hardwareMap.get(DcMotorEx.class, "LF");
        RF = hardwareMap.get(DcMotorEx.class, "RF");
        LB = hardwareMap.get(DcMotorEx.class, "LB");
        RB = hardwareMap.get(DcMotorEx.class, "RB");
        Shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
        Turret = hardwareMap.get(DcMotorEx.class, "Turret");
        Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        Lifter = hardwareMap.get(DcMotorEx.class, "Lifter");
        IntakeServo = hardwareMap.get(Servo.class, "IntakeServo");
        WackerServo = hardwareMap.get(Servo.class, "WackerServo");
        Light = hardwareMap.get(Servo.class, "led");
        intakeSensor = hardwareMap.get(DistanceSensor.class, "intakeSensor");
        imu = hardwareMap.get(IMU.class, "imu");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // --- Motor Setup ---
        LF.setDirection(DcMotor.Direction.REVERSE);
        LB.setDirection(DcMotor.Direction.REVERSE);
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Turret needs BRAKE behavior to stop the high-speed motor from drifting
        Turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        IntakeServo.setPosition(0.75);
        WackerServo.setPosition(0.47);

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));

        limelight.pipelineSwitch(1);
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {
            DriveControl();
            ShooterControl();
            IntakeControl();
            TurretControl();
            AutoIntakeCycle();
            lifterControl();

            if (gamepad1.a || gamepad2.a) {
                threeShots();
            }
            telemetry.update();
        }
    }
    private static final double LIFTER_TICKS_PER_REV = 384.5; // goBILDA 43 RPM
    private static final int LIFTER_TOP = (int)(4 * LIFTER_TICKS_PER_REV);
    private static final int LIFTER_BOTTOM = (int)(7.5 * LIFTER_TICKS_PER_REV);
    private void lifterControl() {
        // Preset positions

        if (gamepad1.dpad_down && gamepad2.dpad_down) {
            moveLifterToPosition(LIFTER_BOTTOM, 1);
        }
        if (gamepad1.dpad_up && gamepad2.dpad_up) {
            moveLifterToPosition(LIFTER_TOP, 1);
        }

        telemetry.addData("Lifter Pos", Lifter.getCurrentPosition());
    }
    private void moveLifterToPosition(int target, double power) {
        Lifter.setTargetPosition(target);
        Lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Lifter.setPower(power);
    }
    private void DriveControl() {
        double forward = applyDeadzone(-gamepad1.left_stick_y, DEADZONE);
        double strafe = applyDeadzone(gamepad1.left_stick_x, DEADZONE);
        double rotate = applyDeadzone(gamepad1.right_stick_x, DEADZONE);

        forward = cubic(forward);
        strafe = cubic(strafe);
        rotate = cubic(rotate) * 0.8;

        double fl = forward + strafe + rotate;
        double fr = forward - strafe - rotate;
        double bl = forward - strafe + rotate;
        double br = forward + strafe - rotate;

        double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
        LF.setPower(fl/max); RF.setPower(fr/max); LB.setPower(bl/max); RB.setPower(br/max);
    }

    private void ShooterControl() {
        // This will set the shooter to low velocity every loop cycle
        // without checking for gamepad inputs.
        Shooter.setVelocity(lowShooterVelocity);
    }

    private void IntakeControl() {
        double power = -1.0;

        // NEW: If we are pressing the lifter controls, force intake to 0
        boolean lifting = (gamepad1.dpad_up && gamepad2.dpad_up) ||
                (gamepad1.dpad_down && gamepad2.dpad_down);

        if (lifting) {
            power = 0;
        } else {
            // Original intake logic
            if (gamepad2.right_trigger > 0.5 && !reversingIntake) {
                reversingIntake = true;
                reverseStartTime = System.currentTimeMillis();
            }

            if (reversingIntake) {
                power = 1.0;
                if (System.currentTimeMillis() - reverseStartTime > 200) {
                    reversingIntake = false;
                }
            }
        }

        Intake.setPower(power);
    }

    private void TurretControl() {
        // Manual override from GP2 stick
        double in = cubic(-gamepad2.left_stick_x);
        Turret.setPower(in * turretPowerScale);
    }

    // 1. Add these variables to your class member variables
    private double lastError = 0;
    private double totalError = 0;
    private long lastTime = 0;

    // 2. Replace your autoAlignTurret with this version
    private void autoAlignTurret() {
        LLResult result = limelight.getLatestResult();

        // --- PID TUNING ---
        // Start with these, then tune in this order: P, then D, then I.
        final double kP = 0.0345;   // Increase until it moves consistently to the target
        final double kI = 0.001;   // Use very sparingly to fix tiny offsets
        final double kD = 0.003;   // Increase to stop oscillation/overshoot

        final double MAX_POWER = 0.8;
        final double DEADZONE = 0.5; // Tighter deadzone since PID is more precise
        // ------------------

        if (result != null && result.isValid()) {
            double error = result.getTx(); // Our target is 0, so error = tx - 0
            long currentTime = System.currentTimeMillis();

            if (lastTime == 0) lastTime = currentTime; // Initialize timer
            double deltaTime = (currentTime - lastTime) / 1000.0;

            if (Math.abs(error) < DEADZONE) {
                Turret.setPower(0);
                totalError = 0; // Reset I
                return;
            }

            // Proportional
            double P = kP * error;

            // Integral (Sum of error over time)
            totalError += error * deltaTime;
            double I = kI * totalError;

            // Derivative (Change in error)
            double derivative = (error - lastError) / deltaTime;
            double D = kD * derivative;

            double power = P + I + D;

            // Cap power and apply
            power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));
            Turret.setPower(-power); // Negative because Limelight Tx is inverted to motor direction

            // Save states for next loop
            lastError = error;
            lastTime = currentTime;
            lastTx = error;
        } else {
            // Tag Lost - Slow down rather than instant stop or hard chase
            Turret.setPower(0);
            totalError = 0;
            lastTime = 0;
        }
    }

    private void threeShots() {
        if (!hasValidTag()) return;
        autoIntakeEnabled = false;
        WackerServo.setPosition(0.47);

        double[] shots = {0.18, 0.57, 0.94};
        for (double pos : shots) {
            IntakeServo.setPosition(pos);
            visionWait(350); // Increased wait slightly for high speed turret to settle
            Wacker();
            visionWait(500);
        }
        Light.setPosition(0.277);
        IntakeServo.setPosition(0.75);
        autoIntakeEnabled = true;
        cycleCount = 0;
        objectDetected = false;
    }

    private void visionWait(long millis) {
        long start = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - start < millis) {
            autoAlignTurret();
            DriveControl();
            telemetry.update();
        }
    }

    private void Wacker() {
        WackerServo.setPosition(0.14);
        sleep(250);
        WackerServo.setPosition(0.47);
    }

    private void AutoIntakeCycle() {
        if (!autoIntakeEnabled) return;
        double d = intakeSensor.getDistance(DistanceUnit.CM);
        if (d > 0.1 && d < 2.0 && !objectDetected && cycleCount < 3) {
            objectDetected = true;
            cycleCount++;
            IntakeCycle();
        } else if (d > 3.0) {
            objectDetected = false;
        }
    }

    private void IntakeCycle() {
        spinnerState = (spinnerState + 1) % 3;
        if (spinnerState == 0) {
            Light.setPosition(.722);
            IntakeServo.setPosition(0.37); }
        else if (spinnerState == 1) {
            Light.setPosition(.611);
            IntakeServo.setPosition(0.0); }
        else {
            Light.setPosition(.5);
            IntakeServo.setPosition(0.18);
            gamepad1.rumble(200);
        }
    }

    private boolean hasValidTag() {
        LLResult result = limelight.getLatestResult();
        return result != null && result.isValid();
    }

    private double applyDeadzone(double v, double dz) { return (Math.abs(v) < dz) ? 0.0 : v; }
    private double cubic(double v) { return v * v * v + 0.35 * v * (1 - Math.abs(v)); }
}