package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Ganymede;
import org.firstinspires.ftc.teamcode.subsystems.PedroDrive;
import org.firstinspires.ftc.teamcode.utils.Constants;

/**
 * ╔═══════════════════════════════════════════════════════════════════════════╗
 * ║                          DRIVE COMMAND                                    ║
 * ║                                                                           ║
 * ║  Default TeleOp driving command that processes gamepad inputs and         ║
 * ║  controls the PedroDrive subsystem for smooth mecanum movement.           ║
 * ╚═══════════════════════════════════════════════════════════════════════════╝
 */
public class Drive extends CommandBase {

    // ============================================================
    //                     COMMAND COMPONENTS
    // ============================================================

    /** Reference to the drive subsystem */
    private final PedroDrive drive;

    /** Gamepad for driver controls */
    private final GamepadEx player1;

    /** Reference to main robot for accessing other subsystems */
    private final Ganymede robot;

    /** FTC Dashboard for real-time telemetry (optional) */
    private final FtcDashboard dashboard;

    // ============================================================
    //                     DRIVE PARAMETERS
    // ============================================================

    /** Current strafe (left/right) speed */
    private double strafeSpeed;

    /** Current forward/backward speed */
    private double forwardSpeed;

    /** Current turning speed */
    private double turnSpeed;

    /**
     * Constructor - Initializes the drive command with required subsystems
     *
     * @param robot Main robot object containing all subsystems
     */
    public Drive(Ganymede robot) {
        // Store references for easy access
        this.robot = robot;
        this.drive = robot.drive;
        this.player1 = robot.player1;

        // Register subsystem requirements for command scheduler
        addRequirements(robot.drive);

        // Initialize FTC Dashboard for advanced telemetry
        dashboard = FtcDashboard.getInstance();
    }

    /**
     * Called once when the command is first scheduled.
     * Use for one-time setup that shouldn't happen in the constructor.
     */
    @Override
    public void initialize() {
        // Reset any drive states if needed
        // Could add initialization telemetry here
        robot.telemetry.addData("Drive", "Command initialized");
    }

    /**
     * Main execution method - called repeatedly while command is scheduled.
     * Processes gamepad inputs and sends drive commands to the subsystem.
     */
    @Override
    public void execute() {

        drive.follower.breakFollowing();
        // ============ SPEED MODIFIERS ============
        // Slow mode is activated when:
        // - Right bumper is held (for precise movements)
        // - Can add other conditions like lift height, arm position, etc.
        double speedMod = robot.opMode.gamepad1.right_bumper
                ? Constants.SLOW_MODE_MULTIPLIER
                : 1.0;  // Full speed

        // ============ GAMEPAD INPUT PROCESSING ============
        // Get raw stick values with proper axis mapping
        // Left stick: forward/strafe movement
        // Right stick: rotation
        double forward = applyDeadZone(player1.getLeftY());
        double strafe = applyDeadZone(player1.getLeftX());
        double turn = applyDeadZone(-player1.getRightX());  // Negative for intuitive turning

        // Debug telemetry for input troubleshooting
        robot.sensors.addTelemetry("══ Drive Command Inputs ══", "");
        robot.sensors.addTelemetry("Left Stick", "X: %.2f Y: %.2f",
                player1.getLeftX(), player1.getLeftY());
        robot.sensors.addTelemetry("Right Stick", "X: %.2f Y: %.2f",
                player1.getRightX(), player1.getRightY());
        robot.sensors.addTelemetry("Speed Mode", speedMod < 1.0 ? "SLOW" : "NORMAL");


        // ============ SEND DRIVE COMMAND ============
        // Apply speed modifier and send to drive subsystem
        drive.drive(
                forward * speedMod,
                strafe * speedMod,
                turn * speedMod
        );

        // Store values for potential telemetry or logging
        this.forwardSpeed = forward * speedMod;
        this.strafeSpeed = strafe * speedMod;
        this.turnSpeed = turn * speedMod;
    }

    /**
     * Called once when the command ends or is interrupted.
     * Use for cleanup operations.
     *
     * @param interrupted True if command was interrupted, false if it finished normally
     */
    @Override
    public void end(boolean interrupted) {
        // Stop the robot when command ends
        drive.stop();

        // Log command termination
        robot.telemetry.addData("Drive", interrupted ? "Interrupted" : "Ended");
    }

    /**
     * Determines if the command has finished executing.
     * For default drive, this never finishes (runs until interrupted).
     *
     * @return Always false - drive command runs continuously
     */
    @Override
    public boolean isFinished() {
        // Drive command runs indefinitely until replaced
        return false;
    }

    // ============================================================
    //                     HELPER METHODS
    // ============================================================

    /**
     * Applies deadzone to gamepad input to prevent drift from stick noise.
     * Any input below the threshold is zeroed to prevent unintended movement.
     *
     * @param input Raw gamepad axis value (-1.0 to 1.0)
     * @return Processed input with deadzone applied
     */
    private double applyDeadZone(double input) {
        // Zero out small inputs to prevent drift
        return Math.abs(input) <= Constants.INPUT_THRESHOLD ? 0.0 : input;
    }

    // ============================================================
    //                     GETTERS (for telemetry/logging)
    // ============================================================

    /** @return Current forward/backward speed after modifiers */
    public double getForwardSpeed() { return forwardSpeed; }

    /** @return Current strafe speed after modifiers */
    public double getStrafeSpeed() { return strafeSpeed; }

    /** @return Current turning speed after modifiers */
    public double getTurnSpeed() { return turnSpeed; }
}