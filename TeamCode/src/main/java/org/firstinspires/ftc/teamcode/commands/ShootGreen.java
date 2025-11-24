package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.Ganymede;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import java.util.concurrent.TimeUnit;

/**
 * ╔═══════════════════════════════════════════════════════════════════════════╗
 * ║                        SHOOT GREEN COMMAND                                ║
 * ║                                                                           ║
 * ║  Alliance-aware command that shoots green balls into the goal.            ║
 * ║  Automatically handles:                                                   ║
 * ║    1. Moving to the correct shooting position for your alliance           ║
 * ║    2. Turning to face the correct shooting angle                          ║
 * ║    3. Positioning the turret to LEFT (for green ball launching)           ║
 * ║    4. Launching the green ball with proper timing                         ║
 * ║                                                                           ║
 * ║  Shooting Positions:                                                      ║
 * ║    • RED alliance:  (77, 88) facing 130°                                  ║
 * ║    • BLUE alliance: (60, 88) facing -132°                                 ║
 * ╚═══════════════════════════════════════════════════════════════════════════╝
 */
public class ShootGreen extends CommandBase {

    // ============================================================
    //                     SHOOTING POSITIONS
    // ============================================================

    /** Red alliance green shooting position */
    private static final double RED_SHOOT_X = 77.0;
    private static final double RED_SHOOT_Y = 88.0;
    private static final double RED_SHOOT_HEADING_DEGREES = 130.0;

    /** Blue alliance green shooting position */
    private static final double BLUE_SHOOT_X = 60.0;
    private static final double BLUE_SHOOT_Y = 88.0;
    private static final double BLUE_SHOOT_HEADING_DEGREES = -132.0;

    // ============================================================
    //                     COMMAND COMPONENTS
    // ============================================================

    private Ganymede robot;
    private Timing.Timer launchTimer;
    private SequentialCommandGroup commandSequence;
    private boolean hasLaunched = false;

    // ============================================================
    //                     CONSTRUCTOR
    // ============================================================

    /**
     * Creates a command to shoot a green ball.
     *
     * Automatically determines the correct shooting position and angle
     * based on robot.isRed alliance color.
     *
     * @param robot Main robot object
     */
    public ShootGreen(Ganymede robot) {
        this.robot = robot;

        // Create timer for launch sequence (similar to LauncherLaunch)
        long timeoutMillis = (long)(10 * 1000);  // 10 second timeout
        this.launchTimer = new Timing.Timer(timeoutMillis, TimeUnit.MILLISECONDS);

        // We'll require the launcher subsystem
        addRequirements(robot.launcher);
    }

    // ============================================================
    //                     COMMAND LIFECYCLE
    // ============================================================

    @Override
    public void initialize() {
        hasLaunched = false;
        launchTimer.start();

        // Determine shooting position based on alliance color
        double shootX = robot.isRed ? RED_SHOOT_X : BLUE_SHOOT_X;
        double shootY = robot.isRed ? RED_SHOOT_Y : BLUE_SHOOT_Y;
        double shootHeadingDegrees = robot.isRed ? RED_SHOOT_HEADING_DEGREES : BLUE_SHOOT_HEADING_DEGREES;

        // Convert heading to radians for Pedro
        double shootHeadingRadians = Math.toRadians(shootHeadingDegrees);

        // Create target pose
        Pose shootingPose = new Pose(shootX, shootY, shootHeadingRadians);

        // Build command sequence:
        // 1. Drive to shooting position with correct heading
        // 2. Rotate turret to LEFT position (for green ball)
        // 3. Small wait to ensure turret is in position
        commandSequence = new SequentialCommandGroup(
                new DriveToPose(robot, shootingPose, 5.0),
                new TurretRotate(robot, Turret.TurretState.LEFT),
                new WaitCommand(500)  // 500ms pause to ensure turret is stable
        );

        // Start the sequence
        commandSequence.schedule();
    }

    @Override
    public void execute() {
        // Telemetry
        robot.sensors.addTelemetry("═══ Shoot Green ═══", "");
        robot.sensors.addTelemetry("Alliance", robot.isRed ? "RED" : "BLUE");

        // Once the movement and turret rotation sequence is complete, start launching
        if (commandSequence.isFinished() && !hasLaunched) {
            // Start launcher motor
            robot.launcher.launcher.setPower(0.9);

            robot.sensors.addTelemetry("Launcher Status", "Spinning up...");
            robot.sensors.addTelemetry("Motor Speed", String.valueOf(robot.launcher.launcher.getVelocity()));
        }

        // After launcher spins up for 2.5 seconds, start feeding green
        if (commandSequence.isFinished() && launchTimer.elapsedTime() >= 2.5 && launchTimer.elapsedTime() < 7.0) {
            robot.launcher.feedGreen();
            robot.sensors.addTelemetry("Launcher Status", "Feeding green ball");
        }

        // Stop feeding at 7 seconds
        if (launchTimer.elapsedTime() >= 7.0 && launchTimer.elapsedTime() < 9.0) {
            robot.launcher.stopFeedingGreen();
            robot.sensors.addTelemetry("Launcher Status", "Feed complete");
        }

        // Mark as complete at 9 seconds
        if (launchTimer.elapsedTime() >= 9.0) {
            hasLaunched = true;
        }
    }

    @Override
    public boolean isFinished() {
        return hasLaunched;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop all launcher operations
        robot.launcher.launcher.setPower(0);
        robot.launcher.stopFeedingGreen();

        if (interrupted) {
            robot.sensors.addTelemetry("Shoot Green", "INTERRUPTED");
        } else {
            robot.sensors.addTelemetry("Shoot Green", "Complete!");
        }
    }
}