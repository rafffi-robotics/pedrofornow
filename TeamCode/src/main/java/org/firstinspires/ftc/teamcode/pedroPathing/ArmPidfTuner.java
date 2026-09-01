package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.telemetry.TelemetryManager;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.methods.pidfMethod;


@Configurable
@TeleOp(name = "ArmPidfTuner")
public class ArmPidfTuner extends OpMode {


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
    private TelemetryManager telemetryM;

    @Override
    public void init() {
        // Change these to match your Robot Configuration.
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        leftArm = hardwareMap.get(DcMotorEx.class, "LeftArm");
        rightArm = hardwareMap.get(DcMotorEx.class, "RightArm");

        controller = new pidfMethod(leftArm, rightArm);
        controller.initMotors();


        ENABLED = false;

        telemetryM.addLine ("Arm PIDF Tuner ready");
        telemetryM.addLine("Set values in Panels before enabling.");

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        telemetryM.update(telemetry);
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

        telemetryM.debug("Enabled", ENABLED);

        telemetryM.debug("Target", controller.getTarget());
        telemetryM.debug("Position", controller.getPosition());
        telemetryM.debug("Error", controller.getError());
        telemetryM.debug("Applied Power", appliedPower);

        telemetryM.debug("kP", kP);
        telemetryM.debug("kI", kI);
        telemetryM.debug("kD", kD);
        telemetryM.debug("kF", kF);

        telemetryM.debug("Left Encoder", leftArm.getCurrentPosition());
        telemetryM.debug("Right Encoder", rightArm.getCurrentPosition());
        telemetryM.debug("Encoder Mismatch", controller.getEncoderMismatch());

        telemetryM.debug("At Target", controller.atTarget());
        telemetryM.debug("Encoder Warning", controller.encodersDisagree());

        telemetryM.update(telemetry);

    }

    @Override
    public void stop() {
        leftArm.setPower(0);
        rightArm.setPower(0);
        controller.reset();
    }
}