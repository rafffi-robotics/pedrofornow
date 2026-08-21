
        package org.firstinspires.ftc.teamcode.methods;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * PIDF position controller for a reverse double 4-bar with TWO encoded motors.
 *
 * Both encoders are read and averaged, which cancels a little noise and lets you
 * detect a slipped belt / stripped gear (see getEncoderMismatch()).
 *
 * ALL GAINS ARE 0 -> the arm will not move until you tune. Notes at the bottom.
 */
public class pidfMethod {

    // ================= TUNE THESE =================
    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0;   // power needed to hold the arm at horizontal

    // ============ MECHANICAL CONSTANTS ============
    public static double TICKS_PER_REV = 537.7;  // gobilda 312rpm; 384.5 for 435rpm
    public static double GEAR_RATIO = 1.0;    // extra reduction AFTER the motor
    public static double ARM_OFFSET_DEG = 0.0;    // bar angle off horizontal when encoder = 0

    public static final double TICKS_PER_DEGREE = (TICKS_PER_REV * GEAR_RATIO) / 360.0;

    // ================ SOFT LIMITS =================
    // set these once you know your real range - stops the PID from driving
    // the linkage into your own frame
    public static int MIN_TICKS = 0;
    public static int MAX_TICKS = 1200;

    // ================== LIMITS ====================
    public static double MAX_POWER = 1.0;
    public static double TOLERANCE_TICKS = 10;
    public static double I_ZONE_TICKS = 100;
    public static double MAX_INTEGRAL = 200;
    public static double MISMATCH_LIMIT = 50;   // ticks of disagreement before you worry

    // ================== STATE =====================
    private final DcMotorEx left;
    private final DcMotorEx right;
    private final ElapsedTime timer = new ElapsedTime();

    private double integralSum = 0;
    private double lastError = 0;
    private double lastDerivative = 0;
    private int target = 0;

    private static final double D_FILTER = 0.8;  // 0 = none, 0.9 = heavy

    public pidfMethod(DcMotorEx left, DcMotorEx right) {
        this.left = left;
        this.right = right;
        timer.reset();
    }

    /**
     * Call once in init(). Reverses the right motor (mirrored mount), zeroes both
     * encoders, and puts both in RUN_WITHOUT_ENCODER so the SDK's internal PID
     * doesn't fight yours.
     */
    public void initMotors() {
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        target = 0;
        reset();
    }

    /**
     * Average of both encoders - this is what the PID controls on.
     */
    public double getPosition() {
        return (left.getCurrentPosition() + right.getCurrentPosition()) / 2.0;
    }

    /**
     * How far apart the two encoders are. Should stay near 0.
     */
    public int getEncoderMismatch() {
        return Math.abs(left.getCurrentPosition() - right.getCurrentPosition());
    }

    /**
     * True if something slipped / one side is stalling. Put this in telemetry.
     */
    public boolean encodersDisagree() {
        return getEncoderMismatch() > MISMATCH_LIMIT;
    }

    public void setTarget(int targetTicks) {
        int clamped = (int) Range.clip(targetTicks, MIN_TICKS, MAX_TICKS);
        if (clamped != this.target) {
            this.target = clamped;
            this.integralSum = 0;
        }
    }

    public int getTarget() {
        return target;
    }

    public double getError() {
        return target - getPosition();
    }

    public boolean atTarget() {
        return Math.abs(getError()) < TOLERANCE_TICKS;
    }

    /**
     * CALL EVERY LOOP. Computes the output and writes it to BOTH motors.
     */
    public double update() {
        double power = calculate();
        left.setPower(power);
        right.setPower(power);
        return power;
    }

    public double calculate() {
        double current = getPosition();
        double error = target - current;

        double dt = timer.seconds();
        if (dt <= 0) dt = 1e-6;
        timer.reset();

        // I with zone + clamp
        if (Math.abs(error) < I_ZONE_TICKS) integralSum += error * dt;
        else integralSum = 0;
        integralSum = Range.clip(integralSum, -MAX_INTEGRAL, MAX_INTEGRAL);

        // filtered D
        double rawD = (error - lastError) / dt;
        double derivative = (D_FILTER * lastDerivative) + ((1 - D_FILTER) * rawD);
        lastError = error;
        lastDerivative = derivative;

        // gravity feedforward off CURRENT angle
        double angleDeg = (current / TICKS_PER_DEGREE) + ARM_OFFSET_DEG;
        double feedforward = kF * Math.cos(Math.toRadians(angleDeg));

        double pid = (Math.abs(error) < TOLERANCE_TICKS)
                ? 0
                : (error * kP) + (integralSum * kI) + (derivative * kD);

        return Range.clip(pid + feedforward, -MAX_POWER, MAX_POWER);
    }

    public void reset() {
        integralSum = 0;
        lastError = 0;
        lastDerivative = 0;
        timer.reset();
    }


}