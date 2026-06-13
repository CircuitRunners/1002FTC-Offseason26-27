package org.firstinspires.ftc.teamcode.allendiffy;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;



import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@Configurable
@TeleOp(name = "Diffy Swerve Module Teleop")
public class DiffyTest extends OpMode {

    // --- Encoder calibration ---
    static final double ENCODER_MAX_VOLTAGE = 3.3;
    static final double ENCODER_OFFSET_DEG = 0; // raw fine-angle offset so wheel-forward = 0
    static final int ENCODER_RATIO = 4;          // 4 encoder rotations per 1 pod rotation

    // --- Tuning constant: angle error (deg) -> turn power ---
    public static double TURN_GAIN = 0.01;

    // --- Test mode targets, editable live from FTC Dashboard ---
    public static boolean TEST_MODE = false;
    public static double TEST_SPEED_PERCENT = 0; // -100 to 100
    public static double TEST_ANGLE_DEG = 0;      // 0-360

    DcMotorEx motorA, motorB;
    AnalogInput azimuthEncoder;
    public ElapsedTime timer = new ElapsedTime();

    private double targetSpeedPercent = 0;
    private double targetAngle = 0;

    // --- Quadrant tracking state ---
    private double lastFineAngle = 0;
    private int quadrant = 0; // 0-3, which 90 deg "slice" the pod is in

    private boolean lastAState = false;

    @Override
    public void init() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }



        motorA = hardwareMap.get(DcMotorEx.class, "motorA");
        motorB = hardwareMap.get(DcMotorEx.class, "motorB");
        azimuthEncoder = hardwareMap.get(AnalogInput.class, "azimuth_enc");

        motorA.setDirection(DcMotorSimple.Direction.REVERSE);

        motorA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // IMPORTANT: physically point the wheel at 0 deg (forward) before init,
        // so quadrant=0 and fine angle ~0 line up with pod angle = 0.
        lastFineAngle = getFineAngle();
        quadrant = 0;
    }

    @Override
    public void loop() {
        timer.reset();

        // --- 1. Get absolute pod angle, 0-360, using quadrant tracking ---
        double currentAngle = getModuleAngle();

        // --- 2. Toggle test mode with the 'A' button ---
        boolean aPressed = gamepad1.a;
        if (aPressed && !lastAState) {
            TEST_MODE = !TEST_MODE;
        }
        lastAState = aPressed;

        // --- 3. Decide where the targets come from ---
        if (TEST_MODE) {
            setTarget(TEST_SPEED_PERCENT, TEST_ANGLE_DEG);
        } else {
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;

            double joystickAngle = Math.toDegrees(Math.atan2(x, y));
            if (joystickAngle < 0) joystickAngle += 360;

            double joystickSpeedPercent = Math.min(Math.hypot(x, y), 1.0) * 100;

            setTarget(joystickSpeedPercent, joystickAngle);
        }

        // --- 4. Convert speed percent -> drive power ---
        double drivePower = Range.clip(targetSpeedPercent / 100.0, -1.0, 1.0);

        // --- 5. Shortest-path error, wrapped to -180..180 ---
        double error = ((targetAngle - currentAngle + 540) % 360) - 180;

        // --- 6. If error > 90 deg, flip wheel direction instead of spinning module 180 ---
        double effectiveTargetAngle = targetAngle;
        if (Math.abs(error) > 90) {
            effectiveTargetAngle = (targetAngle + 180) % 360;
            drivePower = -drivePower;
            error = ((effectiveTargetAngle - currentAngle + 540) % 360) - 180;
        }

        // --- 7. Proportional steering correction ---
        double turnPower = Range.clip(error * TURN_GAIN, -1, 1);

        // --- 8. Diffy swerve kinematics: sum -> drive, difference -> steer ---
        double powerA = drivePower + turnPower;
        double powerB = drivePower - turnPower;

        double max = Math.max(Math.abs(powerA), Math.abs(powerB));
        if (max > 1.0) {
            powerA /= max;
            powerB /= max;
        }

        motorA.setPower(powerA);
        motorB.setPower(powerB);

        telemetry.addData("test mode", TEST_MODE);
        telemetry.addData("quadrant", quadrant);
        telemetry.addData("fine angle (enc/4)", getFineAngle());
        telemetry.addData("current angle", currentAngle);
        telemetry.addData("target angle (raw)", targetAngle);
        telemetry.addData("target angle (effective)", effectiveTargetAngle);
        telemetry.addData("angle error", error);
        telemetry.addData("target speed %", targetSpeedPercent);
        telemetry.addData("drive power", drivePower);
        telemetry.addData("turn power", turnPower);
        telemetry.addData("motorA power", powerA);
        telemetry.addData("motorB power", powerB);
        telemetry.addData("loop time (ms)", timer.milliseconds());
        telemetry.update();
    }

    /**
     * Raw encoder reading converted to 0-90 deg, representing pod angle mod 90.
     * This is unambiguous; it repeats every 90 deg of pod rotation due to the 4:1 ratio.
     */
    private double getFineAngle() {
        double raw = (azimuthEncoder.getVoltage() / ENCODER_MAX_VOLTAGE) * 360.0;
        double fine = (raw - ENCODER_OFFSET_DEG) / ENCODER_RATIO;
        fine = fine % 90;
        if (fine < 0) fine += 90;
        return fine;
    }

    /**
     * Absolute pod angle, 0-360, reconstructed from the fine angle plus a
     * software-tracked quadrant. Requires the module to start at a known
     * orientation (see init()).
     */
    private double getModuleAngle() {
        double fineAngle = getFineAngle();

        // Detect a wrap: fine angle jumped by more than ~45 deg since last loop
        double delta = fineAngle - lastFineAngle;
        if (delta > 45) {
            // wrapped backward (e.g. 5 -> 88): pod moved in negative direction
            quadrant = (quadrant + 3) % 4; // -1 mod 4
        } else if (delta < -45) {
            // wrapped forward (e.g. 88 -> 5): pod moved in positive direction
            quadrant = (quadrant + 1) % 4;
        }

        lastFineAngle = fineAngle;

        return (quadrant * 90.0) + fineAngle;
    }

    /**
     * Set the module's target wheel speed and/or azimuth angle directly.
     * @param speedPercent -100 to 100, percent of max wheel speed (negative = reverse)
     * @param angleDeg     0-360, target module azimuth
     */
    public void setTarget(double speedPercent, double angleDeg) {
        targetSpeedPercent = Range.clip(speedPercent, -100, 100);

        angleDeg = angleDeg % 360;
        if (angleDeg < 0) angleDeg += 360;
        targetAngle = angleDeg;
    }
}