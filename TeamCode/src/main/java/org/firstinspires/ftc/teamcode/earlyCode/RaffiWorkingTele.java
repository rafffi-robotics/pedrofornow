package org.firstinspires.ftc.teamcode.earlyCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class RaffiWorkingTele extends OpMode {
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor intake;
    private CRServo intakeR;
    private CRServo intakeL;
    boolean slowMode = false;

    @Override
    public void init() {
        leftMotor = hardwareMap.get(DcMotor.class, "left");
        rightMotor = hardwareMap.get(DcMotor.class, "right");
        intake = hardwareMap.get(DcMotor.class, "Intake");
        intakeR = hardwareMap.get(CRServo.class, "IntakeR");
        intakeL = hardwareMap.get(CRServo.class, "IntakeL");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);


    }

    @Override
    public void loop() {
        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        double intakePower = gamepad1.right_trigger;
        double extakePower = gamepad1.left_trigger;
        double leftPower = drive + turn;
        double rightPower = drive - turn;
        double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));

        if (gamepad1.aWasPressed()) {
            slowMode = !slowMode;
        }

        if (slowMode) {
            leftPower = leftPower * 0.5;
            rightPower = leftPower * 0.5;
        } else if (max > 1.0) {
            leftPower = leftPower / max;
            rightPower = rightPower / max;
        }
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
        intakeR.setPower(intakePower);

        if (intakePower > 0.5 && extakePower > 0.5) {
            intakeL.setPower(0);
            intakeR.setPower(0);
            intake.setPower(0);
        } else if (intakePower > 0.5) {
            intakeL.setPower(1);
            intakeR.setPower(1);
            intake.setPower(1);

        } else if (extakePower > 0.5) {
            intakeL.setPower(-1);
            intakeR.setPower(-1);
            intake.setPower(-1);

        } else {
            intakeL.setPower(0);
            intakeR.setPower(0);
            intake.setPower(0);
        }
    }
}