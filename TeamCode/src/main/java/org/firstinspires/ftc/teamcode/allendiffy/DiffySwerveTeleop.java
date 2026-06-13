package org.firstinspires.ftc.teamcode.allendiffy;





import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.allendiffy.DiffySwerveModule;
import org.firstinspires.ftc.teamcode.allendiffy.DiffySwerveDrive;

import java.util.List;

@Configurable
@TeleOp(name = "Diffy Swerve Drive + Tune")
public class DiffySwerveTeleop extends OpMode {

    // --- Per-pod encoder zero offsets (raw 0-360 reading when pod points forward) ---
    // Calibrate: set to 0, point pod forward, read "left raw angle" / "right raw angle"
    // in telemetry, then put that value here.
    public static double LEFT_ENCODER_OFFSET = 0;
    public static double RIGHT_ENCODER_OFFSET = 0; // ignored if SECOND_MODULE = false

    // --- Set true for a 2-pod robot (side by side) ---
    public static boolean SECOND_MODULE = true;

    // --- Pod positions relative to robot center (arbitrary consistent units) ---
    // Default: 2 pods side by side, left at x=-1, right at x=+1, both y=0
    public static double LEFT_X = -1, LEFT_Y = 0;
    public static double RIGHT_X = 1, RIGHT_Y = 0;

    // --- Test mode: bypass joysticks, command targets directly from Dashboard ---
    public static boolean TEST_MODE = false;
    public static double TEST_SPEED_LEFT = 0,  TEST_ANGLE_LEFT = 0;
    public static double TEST_SPEED_RIGHT = 0, TEST_ANGLE_RIGHT = 0;

    DiffySwerveDrive drive;
    public ElapsedTime timer = new ElapsedTime();
    private boolean lastAState = false;

    @Override
    public void init() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);


        DiffySwerveModule left = new DiffySwerveModule(
                hardwareMap, "left", "motorA", "motorB", "azimuth_enc", LEFT_ENCODER_OFFSET);

        if (SECOND_MODULE) {
            DiffySwerveModule right = new DiffySwerveModule(
                    hardwareMap, "right", "rightMotorA", "rightMotorB", "rightAzimuth", RIGHT_ENCODER_OFFSET);

            drive = new DiffySwerveDrive(
                    new DiffySwerveModule[]{left, right},
                    new double[]{LEFT_X, RIGHT_X},
                    new double[]{LEFT_Y, RIGHT_Y});
        } else {
            drive = new DiffySwerveDrive(
                    new DiffySwerveModule[]{left},
                    new double[]{LEFT_X},
                    new double[]{LEFT_Y});
        }
    }

    @Override
    public void loop() {
        timer.reset();

        // Toggle test mode with 'A'
        boolean aPressed = gamepad1.a;
        if (aPressed && !lastAState) TEST_MODE = !TEST_MODE;
        lastAState = aPressed;

        if (TEST_MODE) {
            drive.getModule(0).setTarget(TEST_SPEED_LEFT, TEST_ANGLE_LEFT);
            if (SECOND_MODULE) drive.getModule(1).setTarget(TEST_SPEED_RIGHT, TEST_ANGLE_RIGHT);
        } else {
            double vx = gamepad1.left_stick_x;
            double vy = -gamepad1.left_stick_y;
            double omega = -gamepad1.right_stick_x;

            drive.driveRobotCentric(vx, vy, omega);
        }

        drive.update();

        telemetry.addData("TEST_MODE", TEST_MODE);
        for (int i = 0; i < drive.getModuleCount(); i++) {
            DiffySwerveModule m = drive.getModule(i);
            telemetry.addData(m.name + " raw angle", "%.1f", m.getRawEncoderAngle());
            telemetry.addData(m.name + " current angle", "%.1f", m.getCurrentAngle());
            telemetry.addData(m.name + " target angle", "%.1f", m.getTargetAngle());
            telemetry.addData(m.name + " angle error", "%.1f", m.getAngleError());
            telemetry.addData(m.name + " target speed %", "%.1f", m.getTargetSpeedPercent());
            telemetry.addData(m.name + " powerA/B", "%.2f / %.2f", m.getPowerA(), m.getPowerB());
        }
        telemetry.addData("loop time (ms)", timer.milliseconds());
        telemetry.update();
    }

    @Override
    public void stop() {
        drive.stop();
    }
}