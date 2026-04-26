package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
public class IntakeSubsystem {

    // Intake power settings
    public static double closeSpeed = .63;

    public static double autoSpeed = .62;
    public static double farSpeed = .48;
    public static double INTAKE_POWER = 1.0;

    // How long a beam must stay broken to count as "full"
    public static double BEAM_HOLD_TIME = 0.2; // seconds

    private final DcMotorEx intake, transfer;

    // Two break-beam sensors
    private DigitalChannel beam1;
    private DigitalChannel beam2;

    // Timer to measure how long the beam stays broken
    private ElapsedTime beamTimer = new ElapsedTime();

    public IntakeSubsystem(HardwareMap hardwareMap) {

        intake = hardwareMap.get(DcMotorEx.class, "intake1");
        transfer = hardwareMap.get(DcMotorEx.class, "intake2");

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);

        intake.setPower(0);
        transfer.setPower(0);

        // Map break-beam sensors (digital input)
        beam1 = hardwareMap.get(DigitalChannel.class, "beam1");
        beam2 = hardwareMap.get(DigitalChannel.class, "beam2");

        beam1.setMode(DigitalChannel.Mode.INPUT);
        beam2.setMode(DigitalChannel.Mode.INPUT);
    }

    // -----------------------------
    // BASIC INTAKE CONTROLS
    // -----------------------------
    public void intakeFull() {
        intake.setPower(1.0);
        transfer.setPower(1.0);
    }

    public void closeShot() {
        intake.setPower(closeSpeed);
        transfer.setPower(closeSpeed);
    }
    public void farShot() {
        intake.setPower(farSpeed);
        transfer.setPower(farSpeed);
    }
    public void intakeOff() {
        intake.setPower(0);
        transfer.setPower(0);
    }

    public void autoShot() {
        intake.setPower(autoSpeed);
        transfer.setPower(autoSpeed);
    }

    public void intakeReverse() {
        intake.setPower(-INTAKE_POWER);
        transfer.setPower(-INTAKE_POWER);
    }

    // -----------------------------
    // BREAK-BEAM LOGIC
    // -----------------------------

    /**
     * @return true if EITHER break-beam is broken
     */
    private boolean isBeamBroken() {
        // NOTE: Many break-beams return FALSE when broken.
        boolean b1 = !beam1.getState();
        boolean b2 = !beam2.getState();
        return b1 || b2;
    }

    /**
     * @return true if the beam has been broken for > BEAM_HOLD_TIME seconds
     */
    public boolean isFull() {
        if (isBeamBroken()) {
            // If beam is broken, start or continue timing
            if (beamTimer.seconds() > BEAM_HOLD_TIME) {
                return true; // FULL
            }
        } else {
            // Beam not broken → reset timer
            beamTimer.reset();
        }
        return false;
    }
}