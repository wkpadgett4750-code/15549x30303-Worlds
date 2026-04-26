package org.firstinspires.ftc.teamcode.tuners;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Axon_Alignment_Tool", group = "Tuning")
public class AxonAlignmentTool extends LinearOpMode {

    private Servo turretServo1;
    private Servo turretServo2;

    @Override
    public void runOpMode() {
        // Match these names to your hardware config
        turretServo1 = hardwareMap.get(Servo.class, "turret1");
        turretServo2 = hardwareMap.get(Servo.class, "turret2");

        telemetry.addLine("AXON ALIGNMENT TOOL");
        telemetry.addLine("-------------------");
        telemetry.addLine("1. Unlink servos from the turret gears/horns.");
        telemetry.addLine("2. Press PLAY to center servos at 0.5.");
        telemetry.addLine("3. Physically center the turret.");
        telemetry.addLine("4. Re-attach horns/gears.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Lock both servos at the exact midpoint
            turretServo1.setPosition(0.5);
            turretServo2.setPosition(0.5);

            telemetry.addData("Status", "LOCKED AT 0.5");
            telemetry.addData("Servo 1 Pos", turretServo1.getPosition());
            telemetry.addData("Servo 2 Pos", turretServo2.getPosition());
            telemetry.update();
        }
    }
}