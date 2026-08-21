package org.firstinspires.ftc.teamcode.mainTeleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.methods.mecanumDrive;
import org.firstinspires.ftc.teamcode.methods.pidfMethod;

// robot relative w/ intake
@TeleOp()
public class mecanumopmode extends OpMode{

    mecanumDrive drive = new mecanumDrive();
    double forward, strafe, rotate;
    private CRServo intakeR;
    private CRServo intakeL;
    private DcMotor intake;
    private pidfMethod arm;
    static final int ARM_DOWN = 0, ARM_LOW = 230, ARM_HIGH = 900;


    @Override
    public void init()
    {
        //intake = hardwareMap.get(DcMotor.class, "intake");
       drive.init(hardwareMap);
        DcMotorEx l = hardwareMap.get(DcMotorEx.class, "LeftArm");
        DcMotorEx r = hardwareMap.get(DcMotorEx.class, "RightArm ");
        arm = new pidfMethod(l, r);
        arm.initMotors();
    }

    @Override
    public void loop(){
        forward = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        drive.drive(forward, strafe, rotate);

        double intakePower = gamepad2.right_trigger;
        double extakePower = gamepad2.left_trigger;


//        if (intakePower > 0.5 && extakePower > 0.5) {
//            intake.setPower(0);
//        } else if (intakePower > 0.5) {
//           intake.setPower(1);
//
//        } else if (extakePower > 0.5) {
//            intake.setPower(-1);
//
//        } else {
//            intake.setPower(0);
//        }


        if (gamepad2.a) arm.setTarget(ARM_DOWN);
        if (gamepad2.b) arm.setTarget(ARM_LOW);
        if (gamepad2.y) arm.setTarget(ARM_HIGH);

        double power = arm.update();      // must run EVERY loop

        telemetry.addData("target",   arm.getTarget());
        telemetry.addData("pos",      arm.getPosition());
        telemetry.addData("error",    arm.getError());
        telemetry.addData("power",    power);
        telemetry.addData("mismatch", arm.getEncoderMismatch());
        if (arm.encodersDisagree()) telemetry.addLine("!! ENCODERS DISAGREE !!");
        telemetry.update();
    }


}




