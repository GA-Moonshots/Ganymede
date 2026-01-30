package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.follower.Follower;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.Ganymede;
import org.firstinspires.ftc.teamcode.subsystems.PedroDrive;

import java.util.concurrent.TimeUnit;

/**
 * Abstract base class for all drive commands.
 * Handles common elements like robot references and timeout management.
 *
 * All Moon commands automatically require the drive subsystem and include
 * a timeout safety feature to prevent runaway autonomous behavior.
 */
public abstract class DriveAbstract extends CommandBase {

    // ============================================================
    //                     COMMON REFERENCES
    // ============================================================

    /** Main robot object with all subsystems */
    protected final Ganymede robot;

    /** Drive subsystem for movement */
    protected final PedroDrive drive;

    /** Pedro Pathing follower for path execution */
    protected final Follower follower;

    /** Timeout timer to prevent infinite command execution */
    protected final Timing.Timer timer;

    // ============================================================
    //                     CONSTRUCTOR

    /**
     * Creates a new Moon command with timeout safety.
     *
     * Automatically:
     * - Stores robot and drive references
     * - Creates timeout timer (converted from seconds to milliseconds)
     * - Requires the drive subsystem
     *
     * @param robot Main robot object
     * @param timeoutSeconds Maximum time command can run (in seconds)
     *                       Use reasonable values: 3-5s for simple moves,
     *                       10-15s for complex autonomous sequences
     */
    public DriveAbstract(Ganymede robot, double timeoutSeconds) {
        this.robot = robot;
        this.drive = robot.drive;
        this.follower = drive.follower;

        // Convert seconds to milliseconds for the timer
        long timeoutMillis = (long)(timeoutSeconds * 1000);
        this.timer = new Timing.Timer(timeoutMillis, TimeUnit.MILLISECONDS);

        // Ensure drive subsystem is required
        addRequirements(drive);
    }

    // ============================================================
    //                     COMMON UTILITIES
    // ============================================================

    /**
     * Standard cleanup for path-following commands.
     * Call this from your end() method to ensure clean termination.
     */
    protected void standardCleanup() {
        follower.breakFollowing();
        drive.stop();
    }
}