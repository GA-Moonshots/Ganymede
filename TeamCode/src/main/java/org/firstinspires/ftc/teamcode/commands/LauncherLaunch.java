package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Ganymede;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;

/**
 * ╔═══════════════════════════════════════════════════════════════════════════╗
 * ║                        INTAKE LAUNCH COMMAND                              ║
 * ║                                                                           ║
 * ║  Launches a single ball from the launcher when ready.                     ║
 * ║                                                                           ║
 * ║  OPERATION:                                                               ║
 * ║  1. Wait for launcher wheel to reach launch speed (readyToLaunch)         ║
 * ║  2. Open stopper to let one ball pass                                     ║
 * ║  3. Close stopper before another ball enters                              ║
 * ║                                                                           ║
 * ║  TIMING: Opens stopper for ~600ms to allow single ball launch             ║
 * ╚═══════════════════════════════════════════════════════════════════════════╝
 */
public class LauncherLaunch extends CommandBase {

    // ============================================================
    //                    COMMAND COMPONENTS
    // ============================================================

    private Ganymede robot;
    private Launcher launcher;

    // Timer for stopper control
    private ElapsedTime stopperTimer;

    // State tracking
    private boolean stopperOpened;
    private boolean launchComplete;

    // Timing constants (in milliseconds)
    private static final double STOPPER_OPEN_TIME = 300; // Time to let one ball through

    // ============================================================
    //                       CONSTRUCTOR
    // ============================================================

    /**
     * Creates a new IntakeLaunch command.
     *
     * @param robot Main robot instance containing subsystems
     */
    public LauncherLaunch(Ganymede robot) {
        this.robot = robot;
        this.launcher = robot.launcher;

        // Register that this command requires the launcher subsystem
        // This prevents other launcher commands from running simultaneously
        addRequirements(launcher);

        // Initialize timer
        stopperTimer = new ElapsedTime();
    }

    // ============================================================
    //                    COMMAND LIFECYCLE
    // ============================================================

    /**
     * Called once when the command is first scheduled.
     * Resets state variables and prepares for launch sequence.
     */
    @Override
    public void initialize() {
        // Reset state flags
        stopperOpened = false;
        launchComplete = false;

        // Ensure stopper starts closed
        launcher.stopperClosed();
        launcher.launcher.setPower(1);

        // Add telemetry for debugging
        robot.telemetry.addData("IntakeLaunch", "Waiting for launcher ready...");
    }

    /**
     * Called repeatedly while the command is scheduled.
     * Handles the launch sequence timing.
     */
    @Override
    public void execute() {
        // ============ STEP 1: WAIT FOR LAUNCHER READY ============
        // Don't proceed until the launcher wheel is at speed
        if (!launcher.readyToLaunch()) {
            robot.telemetry.addData("IntakeLaunch", "Launcher not ready");
            return; // Keep waiting
        }

        // ============ STEP 2: OPEN STOPPER ============
        // First time we detect launcher is ready, open the stopper
        if (!stopperOpened) {
            launcher.stopperOpen(); // Open stopper
            stopperTimer.reset(); // Start timing
            stopperOpened = true;
            robot.telemetry.addData("IntakeLaunch", "Stopper OPEN - Ball launching!");
        }

        // ============ STEP 3: CLOSE STOPPER AFTER DELAY ============
        // After the ball has passed, close the stopper
        else if (stopperTimer.milliseconds() >= STOPPER_OPEN_TIME && !launchComplete) {
            launcher.stopperClosed(); // Close stopper
            launchComplete = true;
            robot.telemetry.addData("IntakeLaunch", "Stopper CLOSED - Launch complete!");
        }

        // Update telemetry with timing info
        if (stopperOpened && !launchComplete) {
            robot.telemetry.addData("Stopper Timer", "%.0f / %.0f ms",
                    stopperTimer.milliseconds(), STOPPER_OPEN_TIME);
        }
    }

    /**
     * Called once when the command ends (normally or interrupted).
     * Ensures stopper is closed for safety.
     *
     * @param interrupted True if command was interrupted, false if finished normally
     */
    @Override
    public void end(boolean interrupted) {
        // Safety: Always ensure stopper is closed when command ends
        launcher.stopperClosed();
        launcher.launcher.setPower(0);

        if (interrupted) {
            robot.telemetry.addData("IntakeLaunch", "INTERRUPTED - Stopper closed");
        } else {
            robot.telemetry.addData("IntakeLaunch", "Complete - Ball launched!");
        }
    }

    /**
     * Determines if the command has finished executing.
     *
     * @return True when launch sequence is complete (stopper opened and closed)
     */
    @Override
    public boolean isFinished() {
        // Command finishes after stopper has been opened and closed
        return launchComplete;
    }
}