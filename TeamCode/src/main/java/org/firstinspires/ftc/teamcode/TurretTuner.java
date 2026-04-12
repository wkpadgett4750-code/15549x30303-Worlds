package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

@Configurable
@TeleOp(name = "Turret Tuner", group = "Tuning")
public class TurretTuner extends OpMode {

    // ============================
    // PID CONSTANTS (LIVE TUNABLE)
    // ============================
    public static double tp = 0.01;
    public static double ti = 0.0;
    public static double td = 0.0005;

    // Feedforward (dynamic)
    public static double tf = 0.0;

    // Static feedforward (Ts) to overcome friction
    public static double ts = 0.08;

    // ============================
    // TURRET LIMITS (YOUR VALUES)
    // ============================
    public static int turretMin = -8885;
    public static int turretMax = 13594;

    // ============================
    // TURRET SLOPE (YOUR VALUE)
    // ============================
    public static double tSlope = -67.9055; // ticks per degree

    // ============================
    // DPAD INCREMENT (LIVE TUNABLE)
    // ============================
    public static int dpadIncrement = 50;

    // Target position (ticks)
    public static int targetTicks = 0;

    // Hardware
    private CRServo turretLeft, turretRight;
    private DcMotorEx turretEncoder;

    // PID controller
    private PIDController pid;

    @Override
    public void init() {

        turretLeft  = hardwareMap.get(CRServo.class, "turret1");
        turretRight = hardwareMap.get(CRServo.class, "turret2");

        turretEncoder = hardwareMap.get(DcMotorEx.class, "RF");
        turretEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        turretLeft.setDirection(CRServo.Direction.FORWARD);
        turretRight.setDirection(CRServo.Direction.FORWARD);

        pid = new PIDController(tp, ti, td);

        telemetry.addLine("=== TURRET PID TUNER ===");
        telemetry.addLine("Left Stick X = Manual override");
        telemetry.addLine("Dpad Up/Down = Move target by increment");
        telemetry.addLine("Dpad Left/Right = Change increment size");
        telemetry.addLine("A = Reset encoder");
        telemetry.update();
    }

    @Override
    public void loop() {

        // Update PID constants live
        pid.setPID(tp, ti, td);

        // ============================
        // MANUAL OVERRIDE
        // ============================
        double manual = gamepad1.left_stick_x;
        if (Math.abs(manual) > 0.05) {
            turretLeft.setPower(manual);
            turretRight.setPower(manual);
            telemetry.addLine("MANUAL MODE");
            telemetry.update();
            return;
        }

        // ============================
        // DPAD TARGET ADJUSTMENT
        // ============================
        if (gamepad1.dpad_up)    targetTicks += dpadIncrement;
        if (gamepad1.dpad_down)  targetTicks -= dpadIncrement;

        // Change increment size
        if (gamepad1.dpad_right) dpadIncrement += 5;
        if (gamepad1.dpad_left)  dpadIncrement = Math.max(1, dpadIncrement - 5);

        // Clamp target to safe range
        targetTicks = Range.clip(targetTicks, turretMin, turretMax);

        // ============================
        // RESET ENCODER
        // ============================
        if (gamepad1.a) {
            turretEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            turretEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            targetTicks = 0;
        }

        // ============================
        // READ CURRENT POSITION
        // ============================
        int current = turretEncoder.getCurrentPosition();

        // ============================
        // DEGREE-BASED PID + STATIC FF
        // ============================
        double errorTicks = targetTicks - current;
        double errorDegrees = errorTicks / tSlope; // convert ticks → degrees

        double pidOut = pid.calculate(0, errorDegrees);

        // Add static feedforward to overcome friction
        double power = pidOut;

        if (Math.abs(errorDegrees) > 0.5) {
            power += Math.copySign(ts, errorDegrees);
        }

        // Add dynamic feedforward if used
        power += tf;

        // Clamp power
        power = Range.clip(power, -0.6, 0.6);

        // Stop jitter near target
        if (Math.abs(errorDegrees) < 0.5) {
            power = 0;
        }

        // Apply power
        turretLeft.setPower(power);
        turretRight.setPower(power);

        // ============================
        // TELEMETRY
        // ============================
        telemetry.addLine("=== PID DEBUG ===");
        telemetry.addData("Target Ticks", targetTicks);
        telemetry.addData("Current Ticks", current);
        telemetry.addData("Error (ticks)", errorTicks);
        telemetry.addData("Error (degrees)", errorDegrees);
        telemetry.addData("Power", power);
        telemetry.addLine("----------------------");
        telemetry.addData("tp", tp);
        telemetry.addData("ti", ti);
        telemetry.addData("td", td);
        telemetry.addData("tf", tf);
        telemetry.addData("ts", ts);
        telemetry.addLine("----------------------");
        telemetry.addData("Dpad Increment", dpadIncrement);
        telemetry.update();
    }
}