package org.firstinspires.ftc.teamcode.tuners;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class TurrettTunerSubsystem {
    public Servo turretLeft, turretRight;

    // This is your speed limit.
    // Start small (0.003) and increase as you get braver.
    public static double MAX_STEP = 0.005;
    private double currentPos = 0.5;

    public TurrettTunerSubsystem(HardwareMap hardwareMap) {
        turretLeft = hardwareMap.get(Servo.class, "turret1");
        turretRight = hardwareMap.get(Servo.class, "turret2");

        // Assuming they face each other, one usually needs to be reversed
        turretRight.setDirection(Servo.Direction.REVERSE);
    }

    public void setRawPosition(double target) {
        target = Range.clip(target, 0, 1);

        double error = target - currentPos;

        // RAMPING: Prevents the "snap" that grinds gears
        if (Math.abs(error) > MAX_STEP) {
            currentPos += Math.signum(error) * MAX_STEP;
        } else {
            currentPos = target;
        }

        turretLeft.setPosition(currentPos);
        turretRight.setPosition(currentPos);
    }

    public double getCurrentPos() {
        return currentPos;
    }
}