package org.firstinspires.ftc.teamcode;

import java.lang.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.badlogic.gdx.math.Vector2;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Tesseract")
public class Tesseract extends OpMode {

    // Motors
    public DcMotor motorFL;
    public DcMotor motorFR;
    public DcMotor motorBL;
    public DcMotor motorBR;

    public DcMotor[] motorArray;
    public Vector2 leftJoystick = new Vector2();
    public Vector2 rightJoystick = new Vector2();

    void setMotorPowers(double[] motorPowers) {
        // This function assumes the motor powers are ordered: FL, FR, BL, BR. Rewrite to fix later

        for (int i = 0; i < motorArray.length; i++) {
            motorArray[i].setPower(motorPowers[i]);
        }
    }

    // Calc is short for calculator
    double[] CalcDrivetrain(Vector2 leftTargetPos, Vector2 rightTargetPos) {
        // Create variables to hold the x and y positions of each given joystick
        double lTargetX = leftTargetPos.x;
        double lTargetY = leftTargetPos.y;
        double rTargetX = rightTargetPos.x;
        double rTargetY = rightTargetPos.y;

        // Create the variables for each motor output
        double outputFL;
        double outputFR;
        double outputBL;
        double outputBR;

        // Calculate the output values for each motor based on the target values above
        // This isn't the best way to calculate the mecanum drive, so change it later

        outputFL = lTargetY + lTargetX + rTargetX;
        outputFR = lTargetY - lTargetX - rTargetX;
        outputBL = lTargetY - lTargetX + rTargetX;
        outputBR = lTargetY + lTargetX - rTargetX;

        // Create an array with each output and return the output values
        return new double[]{outputFL, outputFR, outputBL, outputBR};
    };

    public double[] motorPowerValues;

    @Override
    public void init() {
        // Map each motor to its corresponding variable
        motorFL = hardwareMap.get(DcMotor.class, "FL");
        motorFR = hardwareMap.get(DcMotor.class, "FR");
        motorBL = hardwareMap.get(DcMotor.class, "BL");
        motorBR = hardwareMap.get(DcMotor.class, "BR");

        // Put all the motors for the drive-chain in an array
        motorArray = new DcMotor[]{motorFL, motorFR, motorBL, motorBR};

        // Set the polarity for each motor
        motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void loop() {

        // Update the Vector2ds for the left and right joystick positions
        leftJoystick.x = gamepad1.left_stick_x;
        leftJoystick.y = gamepad1.left_stick_y;
        rightJoystick.x = gamepad1.right_stick_x;
        rightJoystick.y = gamepad1.right_stick_y;

        // Calculate the power for each motor based on the joystick positions
        motorPowerValues = CalcDrivetrain(leftJoystick, rightJoystick);

        setMotorPowers(motorPowerValues);
    }
}