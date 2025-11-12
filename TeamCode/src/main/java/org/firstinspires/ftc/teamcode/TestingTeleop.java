package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="TestingTeleop")
public class TestingTeleop extends OpMode {

    // Motors
    public DcMotor TestMotor;

    @Override
    public void init() {
        TestMotor = hardwareMap.get(DcMotor.class, "TEST");
    }

    @Override
    public void loop() {
        TestMotor.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
    }
}