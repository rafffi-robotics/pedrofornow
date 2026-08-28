package org.firstinspires.ftc.teamcode.methods;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import com.bylazar.panels.*;

@Configurable
@TeleOp(name = "Arm kF Tuner")
public class tuner extends OpMode {


    // ================= PIDF VALUES =================
    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0;

    // =================== TARGET ====================
    public static int TARGET_TICKS = 0;

    // Keep false until the target and gains are set.
    public static boolean ENABLED = false;

    // =================== LIMITS ====================
    public static double MAX_POWER = 0.5;
    public static double TOLERANCE_TICKS = 10;
    public static double I_ZONE_TICKS = 100;
    public static double MAX_INTEGRAL = 200;

    // Gravity feedforward angle offset.
    public static double ARM_OFFSET_DEG = 0.0;

    private DcMotorEx leftArm;
    private DcMotorEx rightArm;
    private pidfMethod controller;

    @Override
    public void init() {
        // Change these to match your Robot Configuration.
        leftArm = hardwareMap.get(DcMotorEx.class, "LeftArm");
        rightArm = hardwareMap.get(DcMotorEx.class, "RightArm");

        controller = new pidfMethod(leftArm, rightArm);
        controller.initMotors();

        ENABLED = false;

        telemetry.addLine("Arm PIDF Tuner ready");
        telemetry.addLine("Set values in Panels before enabling.");
        telemetry.update();
    }

    @Override
    public void loop() {
        /*
         * Copy the current Panels values into your controller.
         * This happens every loop, so changes apply immediately.
         */
        pidfMethod.kP = kP;
        pidfMethod.kI = kI;
        pidfMethod.kD = kD;
        pidfMethod.kF = kF;

        pidfMethod.MAX_POWER = MAX_POWER;
        pidfMethod.TOLERANCE_TICKS = TOLERANCE_TICKS;
        pidfMethod.I_ZONE_TICKS = I_ZONE_TICKS;
        pidfMethod.MAX_INTEGRAL = MAX_INTEGRAL;
        pidfMethod.ARM_OFFSET_DEG = ARM_OFFSET_DEG;

        controller.setTarget(TARGET_TICKS);

        double appliedPower = 0;

        if (ENABLED) {
            appliedPower = controller.update();
        } else {
            leftArm.setPower(0);
            rightArm.setPower(0);
            controller.reset();
        }

        telemetry.addData("Enabled", ENABLED);
        telemetry.addData("Target", controller.getTarget());
        telemetry.addData("Position", "%.1f", controller.getPosition());
        telemetry.addData("Error", "%.1f", controller.getError());
        telemetry.addData("Power", "%.4f", appliedPower);

        telemetry.addData("kP", "%.6f", kP);
        telemetry.addData("kI", "%.6f", kI);
        telemetry.addData("kD", "%.6f", kD);
        telemetry.addData("kF", "%.6f", kF);

        telemetry.addData("Left encoder", leftArm.getCurrentPosition());
        telemetry.addData("Right encoder", rightArm.getCurrentPosition());
        telemetry.addData(
                "Encoder mismatch",
                controller.getEncoderMismatch()
        );

        telemetry.addData("At target", controller.atTarget());
        telemetry.addData(
                "Encoder warning",
                controller.encodersDisagree()
        );

        telemetry.update();
    }

    @Override
    public void stop() {
        leftArm.setPower(0);
        rightArm.setPower(0);
        controller.reset();
    }
}