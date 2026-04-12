package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.configurables.annotations.Configurable;
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

    // --- HARDWARE ---
    private DcMotorEx shooterLeft, shooterRight;
    private CRServo turretLeft, turretRight;
    private Servo shooterHood, shooterGate;
    private DcMotorEx turretEncoder; // RF Motor Port

    // --- FIELD GEOMETRY ---
    public static double blueGoalX = 6, blueGoalY = 144;
    public static double redGoalX = 138, redGoalY = 144;
    public static double turretOffsetX = -4, turretOffsetY = 0;

    // --- TUNED REGRESSIONS ---
    public static double sSlope = 7.36194;
    public static double sIntercept = 930;
    public static double hSlope = 0.00340002;
    public static double hIntercept = -0.000519631;


    // --- PID & FEEDFORWARD CONSTANTS ---
    public static double kP = 0.01, kI = 0.0, kD = 0, kF = 0.00057;
    public static double RAMP_LIMIT = 0.05;
    public static double tp = 0.0095, ti = 0.000001, td = 0.001, tf = 0.00001, ts = 0.073;
    public static double tSlope = -67.9055;

    // --- LIMITS & SETTINGS ---
    public static int turretMin = -8500, turretMax = 13000;
    public static double hoodMinPos = 0.0, hoodMaxPos = 0.8;
    public static double gateOpenPos = 1.0, gateClosedPos = 0.0;
    public static double MAX_RPM = 2000.0;
    public static double rpmTolerance = 40;
    public static double angleOffset = 180.0;
    public static int autoRPMOffset = -20;
    public static int tOffset = 0; // From example file

    // --- DASHBOARD TUNING VALUES (Restored from your image) ---
    public static int tuningRPM = 2000;
    public static double tuningHoodPos = 0.5;
    public static int tuningTurretTicks = -950;

    // --- STATE VARIABLES ---
    public static int pos = 0;
    public static int vel = 0;
    public static int turretPos;
    public static double turretAngle;
    private double currentTargetVelocity = 0;
    private double lastPower = 0;
    private boolean flywheelsActive = true;
    public static boolean flywheelsEnabled = true;
    public boolean isFiring = false;

    public static PIDController tpidfController;
    private PIDFController flywheelPIDF;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        shooterLeft  = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooter2");
        turretLeft   = hardwareMap.get(CRServo.class, "turret1");
        turretRight  = hardwareMap.get(CRServo.class, "turret2");
        shooterHood  = hardwareMap.get(Servo.class, "hood");
        shooterGate  = hardwareMap.get(Servo.class, "gate");

        turretEncoder = hardwareMap.get(DcMotorEx.class, "RF");
        turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterRight.setDirection(DcMotor.Direction.REVERSE);
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        tpidfController = new PIDController(tp, ti, td);
        flywheelPIDF = new PIDFController(kP, kI, kD, kF);

        closeGate();
    }

    public void update() {
        turretPos = turretEncoder.getCurrentPosition();
    }

    public void setTurretPosition(int targetTicks) {
        int safeTarget = Range.clip(targetTicks, turretMin, turretMax);
        int currentTicks = getTurretPos();

        double errorTicks = currentTicks - safeTarget;
        double errorDegrees = errorTicks / tSlope;

        tpidfController.setPID(tp, ti, td);
        double pidOut = tpidfController.calculate(0, errorDegrees);
        double power = pidOut;

        if (Math.abs(errorDegrees) > 0.5) {
            power += Math.copySign(ts, errorDegrees);
        }

        power += tf;
        power = Range.clip(power, -0.7, 0.7);

        if (Math.abs(errorDegrees) < 0.5) {
            power = 0;
        }

        turretLeft.setPower(power);
        turretRight.setPower(power);
    }

    public void alignTurret(double x, double y, double heading, boolean blue,
                            Telemetry telemetry, double magVel, double thetaVel,
                            double driverOffset, boolean isAuto) {

        double dist = distToGoal(x, y, blue);
        if (dist < 1) return;

        double goalX = blue ? blueGoalX : redGoalX;
        double goalY = blue ? blueGoalY : redGoalY;

        double angleToGoalField = Math.toDegrees(Math.atan2(goalY - y, goalX - x));
        double targetRelative = (angleToGoalField - Math.toDegrees(heading)) + angleOffset + driverOffset;

        double currentAngle = getTurretPos() / tSlope;
        double turretAngleCalculated = currentAngle % 360;
        if (turretAngleCalculated < 0) turretAngleCalculated += 360;
        turretAngle = turretAngleCalculated;

        double diff = targetRelative - turretAngleCalculated;
        if (diff > 180) diff -= 360;
        if (diff < -180) diff += 360;

        pos = (int) ((currentAngle + diff) * tSlope);
        setTurretPosition(pos + tOffset);

        if (flywheelsEnabled && flywheelsActive) {
            int baseRPM = getRegressionRPM(dist);
            vel = isAuto ? baseRPM + autoRPMOffset : baseRPM;
            setFlywheelVelocity(vel, isAuto);
        }

        shooterHood.setPosition(getRegressionHood(dist));
    }

    public void setFlywheelVelocity(double targetRPM, boolean isAuto) {
        targetRPM = Range.clip(targetRPM, 0, MAX_RPM);
        currentTargetVelocity = targetRPM;

        if (targetRPM <= 0) {
            shooterLeft.setPower(0);
            shooterRight.setPower(0);
            lastPower = 0;
            return;
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

    // --- ALL HELPER FUNCTIONS ---
    public int getRegressionRPM(double d) { return (int) (sSlope * d + sIntercept); }
    public double getRegressionHood(double d) { return Range.clip(hSlope * d + hIntercept, hoodMinPos, hoodMaxPos); }

    public void setFlywheelVelocity(int velocity) { setFlywheelVelocity((double) velocity, false); }
    public void setHoodPosition(double pos) { shooterHood.setPosition(Range.clip(pos, hoodMinPos, hoodMaxPos)); }
    public void openGate() { shooterGate.setPosition(gateOpenPos); }
    public void closeGate() { shooterGate.setPosition(gateClosedPos); }

    public void enableFlywheels() { flywheelsActive = true; flywheelsEnabled = true; }
    public void disableFlywheels() {
        flywheelsActive = false;
        flywheelsEnabled = false;
        setFlywheelVelocity(0, false);
    }

    public boolean areFlywheelsEnabled() { return flywheelsActive; }
    public boolean isAtSpeed() { return Math.abs(shooterLeft.getVelocity() - currentTargetVelocity) < rpmTolerance; }

    public double distToGoal(double x, double y, boolean blue) {
        return Math.hypot((blue ? blueGoalX : redGoalX) - x, (blue ? blueGoalY : redGoalY) - y);
    }

    public int getTurretPos() { return turretEncoder.getCurrentPosition(); }
    public double getCurrentVelocity() { return shooterLeft.getVelocity(); }
    public double getTargetVelocity() { return currentTargetVelocity; }
    public double getVel() { return shooterLeft.getVelocity(); }
    public double getTurretAngle() { return turretAngle; }
    public int getPos() { return getTurretPos(); }

    public double getVelError() {
        return Math.abs(getVel() - currentTargetVelocity);
    }

    public void telemetry() {
        // Placeholder for telemetry
    }
}