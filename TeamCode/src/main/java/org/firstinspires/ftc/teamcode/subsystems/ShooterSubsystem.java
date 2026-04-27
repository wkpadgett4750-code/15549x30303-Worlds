package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Configurable
public class ShooterSubsystem {
    private DcMotorEx shooterLeft, shooterRight, rf;
    private Servo turretLeft, turretRight;
    private Servo shooterHood, shooterGate;
    private VoltageSensor batteryVoltageSensor;

    // --- LUT DATA ---
    public static double[] distances = {42.08, 53.03, 64.34, 67.62, 69.52, 74.42, 80, 84.0, 93.5, 102.94, 113.7, 130, 150.86, 154.53, 160.19};
    public static double[] rpms      = {1290, 1300, 1470, 1550, 1550, 1600, 1650, 1660, 1700, 1740, 1770, 1800, 1950, 1960, 1980};
    public static double[] hoods     = {0.70, 0.645, 0.585, 0.56, 0.55, 0.5375, .535, 0.5275, 0.515, 0.51, 0.505, .505, 0.52, 0.52, 0.52};

    // --- TURRET ALIGNMENT CONSTANTS ---
    public static double blueGoalX = 0.0, blueGoalY = 144.0;
    public static double redGoalX = 144.0, redGoalY = 144.0;
    public static double angleOffset = -90.0;
    public static double turretOffsetX = 1.5, turretOffsetY = 0.0;

    // --- NEW TUNED PHYSICS ---
    public static double MAX_SERVO_STEP = 0.02;     // Faster top speed for dampening
    public static double MIN_SERVO_STEP = 0.0025;    // KILLS THE LAG: Minimum crawl speed
    public static double DECEL_THRESHOLD = 0.115;    // Tighter window for snappier tracking
    public static double LONG_MOVE_THRESHOLD = 0.4; // Anything over 40% of the range is a "Long Move"
    public static double DECEL_GAIN = 0.12;          // More aggressive arrival
    public static double DEADZONE = 0.001; // Prevents the turret from vibrating at rest

    // --- CALIBRATED DEGREE MAPPING ---
// Based on: 90 degrees = 0.29 servo units
    public static double turretRangeDegrees = 310.34;
    public static double turretCenterValue = 0.5;

    // Limits based on your 0.00 and 0.95 findings
    public static double minDegreeLimit = -145.0;
    public static double maxDegreeLimit = 115.0;

    // --- DEGREE MAPPING ---
    // Dashboard Inputs
    public static boolean USE_MANUAL_SERVO = false;
    public static double tuningTargetDegrees = 0.0;
    private double currentCommandedPos = 0.5;

    // Flywheel/Hood/Gate Constants
    public static double MAX_RPM = 2000, RAMP_LIMIT = 0.05, kP = 0.001, kF = 0.000427;
    public static double hoodMinPos = 0, hoodMaxPos = .7;
    public static double gateOpenPos = .67, gateClosedPos = .52;
    public static double globalRPMTrim = 0.0;
    public static double tuningRPM = 1500, tuningHoodPos = 0.5;

    public boolean trackingActive = false, flywheelsEnabled = false;
    public double trimDegrees = 0.0;
    private double currentTargetVelocity = 0;
    private double lastFlywheelPower = 0;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        this.turretLeft = hardwareMap.get(Servo.class, "turret1");
        this.turretRight = hardwareMap.get(Servo.class, "turret2");
        this.rf = hardwareMap.get(DcMotorEx.class, "RF");
        this.shooterLeft = hardwareMap.get(DcMotorEx.class, "shooter1");
        this.shooterRight = hardwareMap.get(DcMotorEx.class, "shooter2");
        this.shooterHood = hardwareMap.get(Servo.class, "hood");
        this.shooterGate = hardwareMap.get(Servo.class, "gate");
        this.batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        this.shooterRight.setDirection(DcMotorEx.Direction.REVERSE);

        this.rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.currentCommandedPos = turretCenterValue;
    }

    public void alignTurret(double x, double y, double heading, boolean blue, Telemetry telemetry, double driverOffset, boolean isAuto) {
        if (!trackingActive && !isAuto) return;
        if (USE_MANUAL_SERVO) return;

        double targetX = blue ? blueGoalX : redGoalX;
        double targetY = blue ? blueGoalY : redGoalY;
        double distToRealGoal = Math.hypot(targetX - x, targetY - y);

        if (flywheelsEnabled || isAuto) {
            setFlywheelVelocity(interpolate(distToRealGoal, distances, rpms), isAuto);
            setHoodPosition(interpolate(distToRealGoal, distances, hoods));
        }

        double cos = Math.cos(heading), sin = Math.sin(heading);
        double adjX = x + (turretOffsetX * cos - turretOffsetY * sin);
        double adjY = y + (turretOffsetX * sin + turretOffsetY * cos);

        double angleToGoal = Math.toDegrees(Math.atan2(targetX - adjX, targetY - adjY));
        double targetRel = angleToGoal + Math.toDegrees(heading) - angleOffset + trimDegrees + driverOffset;

        while (targetRel > 180)  targetRel -= 360;
        while (targetRel < -180) targetRel += 360;

        // Switched to Degree-based call
        setTurretPosition(targetRel);
    }

    public void setTurretPosition(double targetDegrees) {
        double safeDegrees = Range.clip(targetDegrees, minDegreeLimit, maxDegreeLimit);
        // Map degrees to 0.0 - 1.0 servo range
        double targetServoPos = (safeDegrees / turretRangeDegrees) + turretCenterValue;
        updateTurretCommand(targetServoPos);
    }

    public void updateTurretCommand(double targetPos) {
        double target = Range.clip(targetPos, 0.0, 0.95);
        double error = target - currentCommandedPos;
        double absError = Math.abs(error);

        if (absError < DEADZONE) return;

        // 1. CHOOSE THE "SNAP" BEHAVIOR
        if (absError > DECEL_THRESHOLD) {
            if (absError > LONG_MOVE_THRESHOLD) {
                // LONG MOVE PROTECTION: If moving >50% of range,
                // start braking WAY earlier (0.25 buffer) to save cables.
                currentCommandedPos = target - (Math.signum(error) * 0.25);
            } else {
                // SHORT MOVE SNAP: Jump closer (0.10 buffer) to target
                // This makes small adjustments feel instant.
                currentCommandedPos = target - (Math.signum(error) * (DECEL_THRESHOLD - 0.01));
            }
        } else {
            // 2. DAMPENER WITH A SPEED FLOOR
            double currentStep = absError * DECEL_GAIN;

            // The MIN_SERVO_STEP here ensures the turret never "crawls" too slow
            currentStep = Range.clip(currentStep, MIN_SERVO_STEP, MAX_SERVO_STEP);

            if (absError > currentStep) {
                currentCommandedPos += Math.signum(error) * currentStep;
            } else {
                currentCommandedPos = target;
            }
        }

        turretLeft.setPosition(currentCommandedPos);
        turretRight.setPosition(currentCommandedPos);
    }
    public void stopTurret() {
        updateTurretCommand(currentCommandedPos);
    }

    // --- FLYWHEEL / HOOD / GATE LOGIC ---
    public void setFlywheelVelocity(double targetRPM, boolean isAuto) {
        targetRPM += globalRPMTrim;
        targetRPM = Math.min(targetRPM, MAX_RPM);
        this.currentTargetVelocity = targetRPM;
        if (targetRPM <= 0) {
            shooterLeft.setPower(0); shooterRight.setPower(0);
            lastFlywheelPower = 0; return;
        }
        double voltage = 12.0;
        try { voltage = batteryVoltageSensor.getVoltage(); } catch (Exception e) {}
        double compensatedFF = (targetRPM * kF) * (12.0 / voltage);
        double error = targetRPM - shooterLeft.getVelocity();
        double requestedPower = compensatedFF + (error * kP);
        if (requestedPower > lastFlywheelPower + RAMP_LIMIT) requestedPower = lastFlywheelPower + RAMP_LIMIT;
        double finalPower = Range.clip(requestedPower, 0, 1.0);
        shooterLeft.setPower(finalPower); shooterRight.setPower(finalPower);
        lastFlywheelPower = finalPower;
    }

    public void setFlywheelVelocity(double targetRPM) { setFlywheelVelocity(targetRPM, false); }

    public double interpolate(double target, double[] xPoints, double[] yPoints) {
        if (target <= xPoints[0]) return yPoints[0];
        if (target >= xPoints[xPoints.length - 1]) return yPoints[yPoints.length - 1];
        for (int i = 0; i < xPoints.length - 1; i++) {
            if (target >= xPoints[i] && target <= xPoints[i + 1]) {
                double fraction = (target - xPoints[i]) / (xPoints[i + 1] - xPoints[i]);
                return yPoints[i] + (yPoints[i + 1] - yPoints[i]) * fraction;
            }
        }
        return yPoints[0];
    }

    public void setHoodPosition(double pos) { shooterHood.setPosition(Range.clip(pos, hoodMinPos, hoodMaxPos)); }
    public void openGate() { shooterGate.setPosition(gateOpenPos); }
    public void closeGate() { shooterGate.setPosition(gateClosedPos); }
    public void enableFlywheels() { flywheelsEnabled = true; }
    public void disableFlywheels() { flywheelsEnabled = false; setFlywheelVelocity(0, false); }
    public double getCurrentVelocity() { return shooterLeft.getVelocity(); }
    public int getTurretPos() { return rf.getCurrentPosition(); }
}