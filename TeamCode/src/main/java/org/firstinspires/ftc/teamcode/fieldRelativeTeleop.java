package org.firstinspires.ftc.teamcode;
// FIELD RELATIVE JV WITH INTAKE
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.methods.mecanumDrive;

@TeleOp()
public class fieldRelativeTeleop extends OpMode{
    mecanumDrive drive = new mecanumDrive();

    double forward, strafe, rotate;
    private CRServo intakeR;
    private CRServo intakeL;
    private DcMotor intake;
    @Override
    public void init(){
        drive.init(hardwareMap);
        intake = hardwareMap.get(DcMotor.class, "intake");
        intakeR = hardwareMap.get(CRServo.class, "IntakeR");
        intakeL= hardwareMap.get(CRServo.class, "IntakeL");

    }
   public void loop(){
       forward = gamepad1.left_stick_y;
       strafe = gamepad1.left_stick_x;
       rotate = gamepad1.right_stick_x;

       drive.drive(forward, strafe, rotate);

       double intakePower = gamepad1.right_trigger;
       double extakePower = gamepad1.left_trigger;


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