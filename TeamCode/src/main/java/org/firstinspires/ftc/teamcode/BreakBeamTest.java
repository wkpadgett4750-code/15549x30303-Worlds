package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "BreakBeamTest")
public class BreakBeamTest extends LinearOpMode {

    private DigitalChannel beam1;
    private DigitalChannel beam2;

    @Override
    public void runOpMode() throws InterruptedException {

        // Hardware mapping
        beam1 = hardwareMap.get(DigitalChannel.class, "beam1");
        beam2 = hardwareMap.get(DigitalChannel.class, "beam2");

        beam1.setMode(DigitalChannel.Mode.INPUT);
        beam2.setMode(DigitalChannel.Mode.INPUT);

        waitForStart();

        while (opModeIsActive()) {

            boolean b1 = beam1.getState();   // TRUE = unbroken
            boolean b2 = beam2.getState();   // TRUE = unbroken

            boolean broken = (!b1) || (!b2); // if either is LOW → broken

            telemetry.addData("Beam 1", b1 ? "GOOD" : "BROKEN");
            telemetry.addData("Beam 2", b2 ? "GOOD" : "BROKEN");

            if (broken) {
                telemetry.addLine("Overall Status: BROKEN");
            } else {
                telemetry.addLine("Overall Status: NOT BROKEN");
            }

            telemetry.update();
        }
    }
}