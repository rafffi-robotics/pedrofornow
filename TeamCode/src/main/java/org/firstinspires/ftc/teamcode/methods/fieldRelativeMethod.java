package org.firstinspires.ftc.teamcode.methods;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class fieldRelativeMethod{
    private DcMotor frontLeftMotor,frontRightMotor, backLeftMotor, backRightMotor;
    private IMU imu;


    public void init(HardwareMap hwMap) {
        frontRightMotor = hwMap.get(DcMotor.class, "FR");
        backRightMotor = hwMap.get(DcMotor.class, "BR");
        frontLeftMotor = hwMap.get(DcMotor.class, "FL");
        backLeftMotor = hwMap.get(DcMotor.class, "BL");


        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        imu = hwMap.get(IMU.class,"IMU");

        RevHubOrientationOnRobot revOrientation = new RevHubOrientationOnRobot(
              RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );
        imu.initialize(new IMU.Parameters(revOrientation));
    }
    public void drive(double forward, double strafe, double rotate) {
        // if these next line were not here strafe and rotate would be reversed up
        strafe = -strafe;
        rotate = -rotate;

        double frontLeftPower = forward + strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backLeftPower = forward - strafe + rotate;
        double backRightPower = forward + strafe - rotate;
        double maxpower = 1.0;
        double maxspeed = 1.0;

        maxpower = Math.max(maxpower , Math.abs(frontLeftPower));
        maxpower = Math.max(maxpower , Math.abs(frontRightPower));
        maxpower = Math.max(maxpower , Math.abs(backLeftPower));
        maxpower = Math.max(maxpower , Math.abs(backRightPower));

        frontLeftMotor.setPower(maxspeed * (frontLeftPower/maxpower));
        frontRightMotor.setPower(maxspeed * (frontRightPower/maxpower));
        backLeftMotor.setPower(maxspeed * (backLeftPower/maxpower));
        backRightMotor.setPower(maxspeed * (backRightPower/maxpower));


    }
    public void drivefeildRelative(double forward, double strafe, double rotate) {
        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(strafe, forward);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newstrafe = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        this.drive(newForward, newstrafe , rotate);

    }
}
