package org.firstinspires.ftc.teamcode.earlyCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled

@TeleOp
public class TankDrive extends OpMode {
    private DcMotor leftmotor;
    private DcMotor rightmotor;
    @Override
    public void init() {

        leftmotor = hardwareMap.get(DcMotor.class, "left");
        rightmotor = hardwareMap.get(DcMotor.class, "right");
        leftmotor.setDirection(DcMotor.Direction.REVERSE);
    }
    @Override
    public void loop() {
double leftpower = -gamepad1.left_stick_y;
double rightpower = -gamepad1.right_stick_y;
leftmotor.setPower(leftpower);
rightmotor.setPower(rightpower);
    }
}

