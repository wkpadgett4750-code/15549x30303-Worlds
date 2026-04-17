package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Configurable
public class ShooterSubsystem {
    // Hardware
    private DcMotorEx shooterLeft, shooterRight, rf;
    private CRServo turretLeft, turretRight;
    private Servo shooterHood, shooterGate;

    // --- LOOK-UP TABLE (LUT) DATA ---
    public static double[] distances = {25, 35, 43.0, 61.1, 79.1, 97.1, 113.7};
    public static double[] rpms      = {1280, 1280, 1280, 1420, 1580, 1650, 1780};
    public static double[] hoods     = {.67, .67, 0.67, 0.62, 0.58, 0.54, 0.52};
    public static double[] tofs      = {.56, .58, 0.58, 0.63, 0.62, 0.65, 0.69};

    // --- TURRET CONSTANTS ---
    public static double blueGoalX = 0.0, blueGoalY = 144.0;
    public static double redGoalX = 0.0, redGoalY = -144.0;
    public static double angleOffset = -90.0;
    public static double tSlope = -67.9055;
    public static double turretOffsetX = 1.5;
    public static double turretOffsetY = 0.0;
    public static int turretMin = -8500, turretMax = 13000;
    public static double ticksPerRev = Math.abs(tSlope * 360.0);

    // Dashboard Tuning
    public static double tuningRPM = 1500, tuningHoodPos = 0.5;
    public static int tuningTurretTicks = 0;

    // --- NEW TUNED TURRET PID VALUES ---
    public static double tp = 0.00011;
    public static double ti = 0.0000048;
    public static double td = 0.0006;
    public static double ts = 0.084;
    public static int deadband = 6;

    // Flywheel Constants
    public static double MAX_RPM = 2800, RAMP_LIMIT = 0.008, rpmTolerance = 50;

    public static double kP = 0.001, kF = 0.00052;
    public static double hoodMinPos = 0, hoodMaxPos = .7;
    public static double gateOpenPos = 0, gateClosedPos = .95;
    public static double globalRPMTrim = 0.0;

    // State
    public boolean trackingActive = false, flywheelsEnabled = false;
    public double trimDegrees = 0.0;
    private double currentTargetVelocity = 0;
    private double lastFlywheelPower = 0;
    private double lastTurretPower = 0;
    private double lastTurretError = 0;
    private double turretIntegralSum = 0;
    private com.qualcomm.robotcore.hardware.VoltageSensor batteryVoltageSensor;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        this.turretLeft = hardwareMap.get(CRServo.class, "turret1");
        this.turretRight = hardwareMap.get(CRServo.class, "turret2");
        this.rf = hardwareMap.get(DcMotorEx.class, "RF");
        this.shooterLeft = hardwareMap.get(DcMotorEx.class, "shooter1");
        this.shooterRight = hardwareMap.get(DcMotorEx.class, "shooter2");
        this.shooterHood = hardwareMap.get(Servo.class, "hood");
        this.shooterGate = hardwareMap.get(Servo.class, "gate");
        this.batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        this.shooterRight.setDirection(DcMotorEx.Direction.REVERSE);
        this.turretLeft.setDirection(CRServo.Direction.REVERSE);
        this.turretRight.setDirection(CRServo.Direction.REVERSE);
        this.rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * SOTM using Interpolation and Vector Compensation
     */
    public void alignTurret(double x, double y, double heading, double vx, double vy, boolean blue, Telemetry telemetry, double driverOffset, boolean isAuto) {
        if (!trackingActive && !isAuto) {
            stopTurret();
            return;
        }

        double targetX = blue ? blueGoalX : redGoalX;
        double targetY = blue ? blueGoalY : redGoalY;

        double distToRealGoal = Math.hypot(targetX - x, targetY - y);
        double timeInFlight = interpolate(distToRealGoal, distances, tofs);

        // Vector Math: Calculate Future Goal (SOTM)
        double futrGoalX = targetX - (vx * timeInFlight);
        double futrGoalY = targetY - (vy * timeInFlight);

        double virtualDist = Math.hypot(futrGoalX - x, futrGoalY - y);

        if (flywheelsEnabled || isAuto) {
            setFlywheelVelocity(interpolate(virtualDist, distances, rpms), isAuto);
            setHoodPosition(interpolate(virtualDist, distances, hoods));
        }

        // World-Locking Turret Math
        double cos = Math.cos(heading), sin = Math.sin(heading);
        double adjX = x + (turretOffsetX * cos - turretOffsetY * sin);
        double adjY = y + (turretOffsetX * sin + turretOffsetY * cos);

        double angleToGoal = Math.toDegrees(Math.atan2(futrGoalX - adjX, futrGoalY - adjY));
        double targetRel = angleToGoal + Math.toDegrees(heading) - angleOffset + trimDegrees + driverOffset;

        while (targetRel > 180)  targetRel -= 360;
        while (targetRel < -180) targetRel += 360;
        int targetTicks = (int) (targetRel * tSlope);

        // FIXED: Continuous wrapping check that respects hard limits
        if (targetTicks < turretMin) {
            if (targetTicks + (int)ticksPerRev <= turretMax) targetTicks += (int)ticksPerRev;
        } else if (targetTicks > turretMax) {
            if (targetTicks - (int)ticksPerRev >= turretMin) targetTicks -= (int)ticksPerRev;
        }

        setTurretPosition(targetTicks);

        if (telemetry != null) {
            telemetry.addData("Dist", virtualDist);
            telemetry.addData("TOF", timeInFlight);
        }
    }

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

    public void setTurretPosition(int targetTicks) {
        int currentTicks = rf.getCurrentPosition();
        int safeTarget = Range.clip(targetTicks, turretMin, turretMax);
        double error = safeTarget - currentTicks;

        if (Math.abs(error) <= deadband) {
            stopTurret();
            return;
        }

        if (Math.abs(error) < 150) {
            turretIntegralSum += error;
        } else {
            turretIntegralSum = 0;
        }

        double derivative = (error - lastTurretError);

        // Tapered Feedforward: Fades out within 50 ticks to stop jitter
        double feedForward = (Math.abs(error) > 80) ? Math.copySign(ts, error) : (error / 80.0) * ts;

        double power = (error * tp) + (turretIntegralSum * ti) + (derivative * td) + feedForward;

        lastTurretError = error;
        lastTurretPower = Range.clip(power, -0.9, 0.9);
        turretLeft.setPower(lastTurretPower);
        turretRight.setPower(lastTurretPower);
    }

    public void setFlywheelVelocity(double targetRPM, boolean isAuto) {
        targetRPM += globalRPMTrim;
        targetRPM = Math.min(targetRPM, MAX_RPM);
        this.currentTargetVelocity = targetRPM;

        if (targetRPM <= 0) {
            shooterLeft.setPower(0);
            shooterRight.setPower(0);
            lastFlywheelPower = 0;
            return;
        }

        double voltage = batteryVoltageSensor.getVoltage();
        // Feedforward with strict voltage compensation
        double compensatedFF = (targetRPM * kF) * (12.0 / voltage);
        double error = targetRPM - shooterLeft.getVelocity();
        double requestedPower = compensatedFF + (error * kP);

        // FUSE PROTECTOR: Limits the RATE of change, not just the max power
        if (requestedPower > lastFlywheelPower + RAMP_LIMIT) {
            requestedPower = lastFlywheelPower + RAMP_LIMIT;
        }

        double finalPower = Range.clip(requestedPower, 0, 1.0);
        shooterLeft.setPower(finalPower);
        shooterRight.setPower(finalPower);
        lastFlywheelPower = finalPower;
    }
    public void setHoodPosition(double pos) {
        shooterHood.setPosition(Range.clip(pos, hoodMinPos, hoodMaxPos));
    }

    public void stopTurret() {
        turretLeft.setPower(0);
        turretRight.setPower(0);
        turretIntegralSum = 0;
    }

    public void openGate() { shooterGate.setPosition(gateOpenPos); }
    public void closeGate() { shooterGate.setPosition(gateClosedPos); }
    public void enableFlywheels() { flywheelsEnabled = true; }
    public void disableFlywheels() { flywheelsEnabled = false; setFlywheelVelocity(0, false); }
    public double getCurrentVelocity() { return shooterLeft.getVelocity(); }
    public int getTurretPos() { return rf.getCurrentPosition(); }
    public boolean isAtSpeed() { return Math.abs(getCurrentVelocity() - currentTargetVelocity) < rpmTolerance; }
}