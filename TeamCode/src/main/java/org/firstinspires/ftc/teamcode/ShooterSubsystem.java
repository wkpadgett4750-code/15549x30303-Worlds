package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Configurable
public class ShooterSubsystem {
    private DcMotorEx shooterLeft, shooterRight;
    private CRServo turretLeft, turretRight;
    private Servo shooterHood, shooterGate;
    private AnalogInput turretEncoderLeft;

    // --- FIELD GOALS ---
    public static double blueGoalX = 0, blueGoalY = 144, redGoalX = 144, redGoalY = 144;
    public static double turretOffsetX = -4.5, turretOffsetY = 0;

    // --- YOUR TUNED LINEAR REGRESSIONS ---
    public static double sSlope = 7.36194;
    public static double sIntercept = 930; // Updated to 890 per request

    public static double hSlope = 0.00340002;
    public static double hIntercept = -0.000519631;

    // --- TUNING SLIDERS (Restored) ---
    public static int tuningRPM = 2000;
    public static double tuningHoodPos = 0.5;
    public static int tuningTurretTicks = -950;

    // --- SAFETY LIMITS ---
    public static double MAX_RPM = 2000.0;
    public static double hoodMinPos = 0.0, hoodMaxPos = 0.8;
    public static int autoRPMOffset = -20;
    public static double rpmTolerance = 40;
    public boolean isFiring = false;

    // --- PID ---
    public static double kP = 0.01, kI = 0.0, kD = 0, kF = 0.00057, RAMP_LIMIT = 0.05;
    public static double tp = 0.0025, ti = 0, td = 0.00008, tf = 0;
    public static double tSlope = -5.5617977528;
    public static int turretMin = -1195, turretMax = -700;
    public static double angleOffset = 180.0, gateOpenPos = 1.0, gateClosedPos = 0.0, gateToFeedDelay = 0.4;

    private boolean flywheelsActive = true;
    public static boolean flywheelsEnabled = true;
    private double currentTargetVelocity = 0;
    private double lastPower = 0;
    private PIDFController flywheelPIDF;
    public static PIDController tpidfController;

    private static double lastRawDegrees = -1;
    private static double totalUnwrappedDegrees = 0;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooter2");
        turretLeft = hardwareMap.get(CRServo.class, "turret1");
        turretRight = hardwareMap.get(CRServo.class, "turret2");
        shooterHood = hardwareMap.get(Servo.class, "hood");
        shooterGate = hardwareMap.get(Servo.class, "gate");
        //turretEncoder = hardwareMap.get(AnalogInput.class, "turretEncoderLeft");

        shooterLeft.setDirection(DcMotor.Direction.REVERSE);
        shooterRight.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        turretLeft.setDirection(CRServo.Direction.FORWARD);
        turretRight.setDirection(CRServo.Direction.FORWARD);

        tpidfController = new PIDController(tp, ti, td);
        flywheelPIDF = new PIDFController(kP, kI, kD, kF);

        double currentRaw = (turretEncoderLeft.getVoltage() / 3.3) * 360.0;
        if (lastRawDegrees == -1) {
            lastRawDegrees = currentRaw;
            totalUnwrappedDegrees = currentRaw;
        }
        closeGate();
    }
    /*
        public void alignTurret(double x, double y, double heading, boolean blue, Telemetry telemetry, double magVel, double thetaVel, double driverOffset, boolean isAuto) {
            double dist = distToGoal(x, y, blue);
            if (dist < 1) return;

            double angleToGoalField = Math.toDegrees(Math.atan2((blue?blueGoalY:redGoalY) - y, (blue?blueGoalX:redGoalX) - x));
            double targetRelative = (angleToGoalField - Math.toDegrees(heading)) + angleOffset + driverOffset;

            double currentAngle = totalUnwrappedDegrees;
            double diff = targetRelative - (currentAngle % 360);
            if (diff > 180) diff -= 360; if (diff < -180) diff += 360;

            setTurretPosition((int) ((totalUnwrappedDegrees + diff) * tSlope));

            if (flywheelsEnabled && flywheelsActive) {
                int baseRPM = getRegressionRPM(dist);
                setFlywheelVelocity(isAuto ? baseRPM + autoRPMOffset : baseRPM, isAuto);
            } else {
                setFlywheelVelocity(0, isAuto);
            }

            shooterHood.setPosition(getRegressionHood(dist));
        }
    */
    public void setFlywheelVelocity(double targetRPM, boolean isAuto) {
        targetRPM = Math.min(targetRPM, MAX_RPM);
        this.currentTargetVelocity = targetRPM;

        if (targetRPM <= 0) {
            shooterLeft.setPower(0); shooterRight.setPower(0);
            lastPower = 0; return;
        }

        if (isAuto) {
            if (shooterLeft.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            shooterLeft.setVelocity(targetRPM);
            shooterRight.setVelocity(targetRPM);
        } else {
            if (shooterLeft.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
                shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            flywheelPIDF.setPIDF(kP, kI, kD, kF);
            double currentVel = shooterLeft.getVelocity();
            double pidfPower = flywheelPIDF.calculate(currentVel, targetRPM);
            if (pidfPower > lastPower + RAMP_LIMIT) pidfPower = lastPower + RAMP_LIMIT;
            double finalPower = Range.clip(pidfPower, 0, 1.0);
            shooterLeft.setPower(finalPower);
            shooterRight.setPower(finalPower);
            lastPower = finalPower;
        }
    }

    // --- LINEAR MATH HELPERS ---
    public int getRegressionRPM(double d) { return (int) (sSlope * d + sIntercept); }
    public double getRegressionHood(double d) { return Range.clip(hSlope * d + hIntercept, hoodMinPos, hoodMaxPos); }

    // --- MANUAL SETTERS ---
    public void setFlywheelVelocity(int velocity) { setFlywheelVelocity((double)velocity, false); }
    public void setHoodPosition(double pos) { shooterHood.setPosition(Range.clip(pos, hoodMinPos, hoodMaxPos)); }
/*
    public void setTurretPosition(int targetTicks) {
        int currentTicks = getTurretPos();
        int safeTarget = Range.clip(targetTicks, turretMin, turretMax);
        double power = Range.clip(tpidfController.calculate(currentTicks, safeTarget) + tf, -0.7, 0.7);
        if (Math.abs(safeTarget - currentTicks) < 5) power = 0;
        turretLeft.setPower(power); turretRight.setPower(power);
    }

    public int getTurretPos() {
        double currentRaw = (turretEncoderLeft.getVoltage() / 3.3) * 360.0;
        double delta = currentRaw - lastRawDegrees;
        if (delta > 180) delta -= 360; else if (delta < -180) delta += 360;
        totalUnwrappedDegrees += delta; lastRawDegrees = currentRaw;
        return (int) (totalUnwrappedDegrees * tSlope);
    }



    */


    public void openGate() { shooterGate.setPosition(gateOpenPos); }
    public void closeGate() { shooterGate.setPosition(gateClosedPos); }
    public void enableFlywheels() { flywheelsActive = true; flywheelsEnabled = true; }
    public void disableFlywheels() { flywheelsActive = false; flywheelsEnabled = false; setFlywheelVelocity(0, false); }
    public boolean areFlywheelsEnabled() { return flywheelsActive; }
    public boolean isAtSpeed() { return Math.abs(shooterLeft.getVelocity() - currentTargetVelocity) < rpmTolerance; }
    public double distToGoal(double x, double y, boolean blue) { return Math.hypot((blue ? blueGoalX : redGoalX) - x, (blue ? blueGoalY : redGoalY) - y); }
    public double getCurrentVelocity() { return shooterLeft.getVelocity(); }
    public double getTargetVelocity() { return currentTargetVelocity; }
}