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
 * ║                        SHOOT PURPLE COMMAND                               ║
 * ║                                                                           ║
 * ║  Alliance-aware command that shoots purple balls into the goal.           ║
 * ║                                                                           ║
 * ║  Shooting Positions:                                                      ║
 * ║    • RED alliance:  (77, 88) facing 43°                                   ║
 * ║    • BLUE alliance: (60, 88) facing 135°                                  ║
 * ║                                                                           ║
 * ║  Timing Sequence (aligned with LauncherLaunch):                           ║
 * ║    0.0s - 2.0s : Launcher spins up                                        ║
 * ║    2.0s - 5.0s : Feed purple ball                                         ║
 * ║    5.0s - 7.0s : Stop feeding, let ball clear                             ║
 * ║    7.0s+       : Complete                                                 ║
 * ╚═══════════════════════════════════════════════════════════════════════════╝
 */
public class ShootPurple extends CommandBase {

    // ============================================================
    //                     SHOOTING POSITIONS
    // ============================================================

    /** Red alliance purple shooting position */
    private static final double RED_SHOOT_X = 77.0;
    private static final double RED_SHOOT_Y = 88.0;
    private static final double RED_SHOOT_HEADING_DEGREES = 43.0;

    /** Blue alliance purple shooting position */
    private static final double BLUE_SHOOT_X = 60.0;
    private static final double BLUE_SHOOT_Y = 88.0;
    private static final double BLUE_SHOOT_HEADING_DEGREES = 135.0;

    // ============================================================
    //                     TIMING CONSTANTS
    // ============================================================
    // Aligned with LauncherLaunch timing

    /** Time to spin up launcher before feeding (seconds) */
    private static final double SPINUP_TIME = 2.0;
    /** Time to stop feeding (seconds after sequence complete) */
    private static final double FEED_STOP_TIME = 5.0;
    /** Time when command is complete (seconds after sequence complete) */
    private static final double COMPLETE_TIME = 7.0;
    /** Overall safety timeout (seconds) */
    private static final long SAFETY_TIMEOUT = 15;

    // ============================================================
    //                     COMMAND COMPONENTS
    // ============================================================

    private final Ganymede robot;
    private final Timing.Timer safetyTimer;
    private SequentialCommandGroup commandSequence;

    // ============================================================
    //                     STATE TRACKING
    // ============================================================

    /** Tracks when the drive/turret sequence has completed */
    private boolean sequenceComplete = false;
    /** Timestamp when the sequence completed (milliseconds) */
    private long sequenceCompleteTime = 0;
    /** Tracks when the full launch cycle is done */
    private boolean launchComplete = false;

    // ============================================================
    //                     CONSTRUCTOR
    // ============================================================

    public ShootPurple(Ganymede robot) {
        this.robot = robot;
        this.safetyTimer = new Timing.Timer(SAFETY_TIMEOUT, TimeUnit.SECONDS);

        // Require both subsystems since we drive AND launch
        addRequirements(robot.launcher, robot.drive);
    }

    // ============================================================
    //                     COMMAND LIFECYCLE
    // ============================================================

    @Override
    public void initialize() {
        // Reset state
        sequenceComplete = false;
        sequenceCompleteTime = 0;
        launchComplete = false;

        // Start safety timer
        safetyTimer.start();

        // Determine shooting position based on alliance color
        double shootX = robot.isRed ? RED_SHOOT_X : BLUE_SHOOT_X;
        double shootY = robot.isRed ? RED_SHOOT_Y : BLUE_SHOOT_Y;
        double shootHeadingDegrees = robot.isRed ? RED_SHOOT_HEADING_DEGREES : BLUE_SHOOT_HEADING_DEGREES;
        double shootHeadingRadians = Math.toRadians(shootHeadingDegrees);

        Pose shootingPose = new Pose(shootX, shootY, shootHeadingRadians);

        // Build command sequence
        commandSequence = new SequentialCommandGroup(
                new DriveToPose(robot, shootingPose, 5.0),
                new TurretRotate(robot, Turret.TurretState.FRONT),
                new WaitCommand(500)
        );

        commandSequence.schedule();
    }

    @Override
    public void execute() {
        // Telemetry header
        robot.sensors.addTelemetry("═══ Shoot Purple ═══", "");
        robot.sensors.addTelemetry("Alliance", robot.isRed ? "RED" : "BLUE");

        // Check if sequence just completed
        if (!sequenceComplete && commandSequence.isFinished()) {
            sequenceComplete = true;
            sequenceCompleteTime = System.currentTimeMillis();
        }

        // If sequence isn't done yet, show waiting status
        if (!sequenceComplete) {
            robot.sensors.addTelemetry("Status", "Positioning...");
            return;
        }

        // Calculate time since sequence completed (in seconds)
        double timeSinceReady = (System.currentTimeMillis() - sequenceCompleteTime) / 1000.0;
        robot.sensors.addTelemetry("Launch Timer", "%.1f s", timeSinceReady);

        // Always run launcher at full power once positioned
        robot.launcher.launcher.setPower(1.0);

        // Phase 1: Spin up only (0 to SPINUP_TIME)
        if (timeSinceReady < SPINUP_TIME) {
            robot.sensors.addTelemetry("Launcher Status", "Spinning up...");
            robot.sensors.addTelemetry("Motor Speed", "%.0f", robot.launcher.launcher.getVelocity());
        }
        // Phase 2: Feed purple ball (SPINUP_TIME to FEED_STOP_TIME)
        else if (timeSinceReady < FEED_STOP_TIME) {
            robot.launcher.feedPurple();  // Uses -1 power after Launcher.java fix
            robot.sensors.addTelemetry("Launcher Status", "Feeding purple ball");
            robot.sensors.addTelemetry("Motor Speed", "%.0f", robot.launcher.launcher.getVelocity());
        }
        // Phase 3: Stop feeding, let ball clear (FEED_STOP_TIME to COMPLETE_TIME)
        else if (timeSinceReady < COMPLETE_TIME) {
            robot.launcher.stopFeedingPurple();
            robot.sensors.addTelemetry("Launcher Status", "Ball clearing...");
        }
        // Phase 4: Complete
        else {
            launchComplete = true;
            robot.sensors.addTelemetry("Launcher Status", "Complete!");
        }
    }

    @Override
    public boolean isFinished() {
        return launchComplete || safetyTimer.done();
    }

    @Override
    public void end(boolean interrupted) {
        // Cancel the command sequence if still running
        if (!commandSequence.isFinished()) {
            commandSequence.cancel();
        }

        // Stop all launcher operations
        robot.launcher.stopAll();

        if (interrupted) {
            robot.sensors.addTelemetry("Shoot Purple", "INTERRUPTED");
        } else if (safetyTimer.done() && !launchComplete) {
            robot.sensors.addTelemetry("Shoot Purple", "TIMEOUT");
        } else {
            robot.sensors.addTelemetry("Shoot Purple", "Complete!");
        }
    }
}