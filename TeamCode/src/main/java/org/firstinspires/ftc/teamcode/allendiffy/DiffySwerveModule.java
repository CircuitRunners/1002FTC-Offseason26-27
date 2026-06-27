package org.firstinspires.ftc.teamcode.allendiffy;




import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

/**
 * One diffy swerve pod: 2 drive motors + 1 absolute analog encoder (4:1 ratio to pod).
 * Handles angle tracking (with quadrant unwrap), shortest-path optimization,
 * and sum/diff motor mixing.
 */
public class DiffySwerveModule {

    // Shared steering gain - tune via FTC Dashboard
    public static double TURN_GAIN = 0.01;

    public static double ENCODER_RATIO = 2; // 4 encoder rotations per 1 pod rotation

    private final DcMotorEx motorA, motorB;
    private final AnalogInput azimuthEncoder;
    private final double encoderOffsetDeg; // raw 0-360 encoder reading when pod points forward

    public final String name;

    // Quadrant tracking state
    private double lastFineAngle;
    private int quadrant;

    // Current targets
    private double targetSpeedPercent = 0;
    private double targetAngleDeg = 0;

    // Latest computed values, exposed for telemetry
    private double currentAngle, angleError, drivePower, turnPower, powerA, powerB;

    public DiffySwerveModule(HardwareMap hw, String name, String motorAName, String motorBName,
                             String encoderName, double encoderOffsetDeg) {
        this.name = name;
        this.encoderOffsetDeg = encoderOffsetDeg;

        motorA = hw.get(DcMotorEx.class, motorAName);
        motorB = hw.get(DcMotorEx.class, motorBName);
        azimuthEncoder = hw.get(AnalogInput.class, encoderName);

        motorA.setDirection(DcMotorSimple.Direction.REVERSE);

        motorA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetQuadrantTracking();
    }

    /** Call during init, with the pod PHYSICALLY pointed at its 0 deg (forward) reference. */
    public void resetQuadrantTracking() {
        lastFineAngle = getFineAngle();
        quadrant = 0;
    }

    /**
     * Set target wheel speed (-100 to 100 %) and azimuth angle (0-360 deg).
     * Call directly for bench testing, or let DiffySwerveDrive call it from kinematics.
     */
    public void setTarget(double speedPercent, double angleDeg) {
        targetSpeedPercent = Range.clip(speedPercent, -100, 100);
        angleDeg %= 360;
        if (angleDeg < 0) angleDeg += 360;
        targetAngleDeg = angleDeg;
    }

    /** Reads encoder, computes steering correction, sets motor powers. Call every loop. */
    public void update() {
        currentAngle = getModuleAngle();

        drivePower = Range.clip(targetSpeedPercent / 100.0, -1.0, 1.0);
        double target = targetAngleDeg;

        angleError = wrapError(target - currentAngle);

        // Shortest-path: if error > 90 deg, spin module the other way and reverse wheel
        if (Math.abs(angleError) > 90) {
            target = (target + 180) % 360;
            drivePower = -drivePower;
            angleError = wrapError(target - currentAngle);
        }

        turnPower = Range.clip(angleError * TURN_GAIN, -1, 1);

        // Diffy swerve mixing: sum -> drive, difference -> steer
        powerA = drivePower + turnPower;
        powerB = drivePower - turnPower;

        double max = Math.max(Math.abs(powerA), Math.abs(powerB));
        if (max > 1.0) {
            powerA /= max;
            powerB /= max;
        }

        motorA.setPower(powerA);
        motorB.setPower(powerB);
    }

    public void stop() {
        setTarget(0, targetAngleDeg);
        motorA.setPower(0);
        motorB.setPower(0);
    }

    private double wrapError(double error) {
        return ((error + 540) % 360) - 180;
    }

    /** Raw encoder angle, 0-360, BEFORE dividing by the 4:1 ratio. Useful for calibration. */
    public double getRawEncoderAngle() {
        return (int) ((Math.round(azimuthEncoder.getVoltage() / 3.2 * 360)) % 360);
    }

    /** Encoder reading mod 90 deg - unambiguous, but repeats every 90 deg of pod rotation. */
    private double getFineAngle() {
        double fine = (getRawEncoderAngle() - encoderOffsetDeg) / ENCODER_RATIO;
        fine %= 90;
        if (fine < 0) fine += 90;
        return fine;
    }

    /** Absolute pod angle, 0-360, from fine angle + software-tracked quadrant. */
    private double getModuleAngle() {
        double fineAngle = getFineAngle();
        double delta = fineAngle - lastFineAngle;

        if (delta > 45) quadrant = (quadrant + 3) % 4;       // wrapped backward
        else if (delta < -45) quadrant = (quadrant + 1) % 4; // wrapped forward

        lastFineAngle = fineAngle;
        return (quadrant * 90.0) + fineAngle;
    }

    // --- Telemetry getters ---
    public double getCurrentAngle() { return currentAngle; }
    public double getTargetAngle() { return targetAngleDeg; }
    public double getAngleError() { return angleError; }
    public double getTargetSpeedPercent() { return targetSpeedPercent; }
    public double getDrivePower() { return drivePower; }
    public double getTurnPower() { return turnPower; }
    public double getPowerA() { return powerA; }
    public double getPowerB() { return powerB; }
}
