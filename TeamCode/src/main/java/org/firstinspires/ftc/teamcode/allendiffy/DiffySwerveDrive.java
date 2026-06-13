package org.firstinspires.ftc.teamcode.allendiffy;



/**
 * Robot-centric kinematics for 1-2 diffy swerve modules.
 * Each module is defined by its (x, y) position relative to the robot's center of rotation.
 * Units don't matter as long as they're CONSISTENT - results get re-normalized to fit
 * the -100..100% motor range regardless of scale.
 */
public class DiffySwerveDrive {

    private final DiffySwerveModule[] modules;
    private final double[] moduleX, moduleY;

    /**
     * @param modules  array of 1 or 2 modules
     * @param moduleX  x position of each module relative to robot center (same order)
     * @param moduleY  y position of each module relative to robot center (same order)
     */
    public DiffySwerveDrive(DiffySwerveModule[] modules, double[] moduleX, double[] moduleY) {
        this.modules = modules;
        this.moduleX = moduleX;
        this.moduleY = moduleY;
    }

    /**
     * Robot-centric drive. vx/vy = translation (-1 to 1, robot's own frame, +y = forward,
     * +x = right). omega = rotation (-1 to 1, + = CCW).
     *
     * For a module centered at (0,0), omega has no effect - a pod sitting exactly at the
     * robot's center of rotation can't impart any spin on its own.
     */
    public void driveRobotCentric(double vx, double vy, double omega) {
        double[] velX = new double[modules.length];
        double[] velY = new double[modules.length];
        double maxMag = 0;

        for (int i = 0; i < modules.length; i++) {
            // v = vTranslation + omega x r, where r = (moduleX, moduleY)
            velX[i] = vx - omega * moduleY[i];
            velY[i] = vy + omega * moduleX[i];
            maxMag = Math.max(maxMag, Math.hypot(velX[i], velY[i]));
        }

        // If any module's required speed exceeds 100%, scale ALL of them down together
        // so the robot still moves in the commanded direction, just slower
        double scale = (maxMag > 1.0) ? 1.0 / maxMag : 1.0;

        for (int i = 0; i < modules.length; i++) {
            double speedPercent = Math.hypot(velX[i], velY[i]) * scale * 100;
            double angleDeg = Math.toDegrees(Math.atan2(velX[i], velY[i]));
            if (angleDeg < 0) angleDeg += 360;

            modules[i].setTarget(speedPercent, angleDeg);
        }
    }

    /** Sends current targets to motors. Call every loop after driveRobotCentric()/setTarget(). */
    public void update() {
        for (DiffySwerveModule m : modules) m.update();
    }

    public void stop() {
        for (DiffySwerveModule m : modules) m.stop();
    }

    public DiffySwerveModule getModule(int index) { return modules[index]; }
    public int getModuleCount() { return modules.length; }
}
