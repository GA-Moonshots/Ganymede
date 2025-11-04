package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Ganymede;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Turret.TurretState;
import org.firstinspires.ftc.teamcode.utils.Constants;

/**
 * ╔═══════════════════════════════════════════════════════════════════════════╗
 * ║                        TURRET ROTATE COMMAND                              ║
 * ║                                                                           ║
 * ║  Rotates the turret to a specified position (FRONT or LEFT) using          ║
 * ║  touch sensors as limit switches. Sets state to MOVING during             ║
 * ║  rotation, then to target state when sensor is triggered.                 ║
 * ╚═══════════════════════════════════════════════════════════════════════════╝
 */
public class TurretRotate extends CommandBase {

    private Ganymede robot;
    private Turret turret;

    /** Target position for the turret */
    private TurretState targetState;

    /**
     * Creates a new TurretRotate command
     *
     * @param robot Main robot instance
     * @param targetState Desired turret position (FRONT or LEFT)
     */
    public TurretRotate(Ganymede robot, TurretState targetState) {
        this.robot = robot;
        this.turret = robot.turret;
        this.targetState = targetState;

        // Register subsystem requirements for command scheduler
        addRequirements(turret);
    }

    /**
     * Called once when command is scheduled.
     * Sets turret state to MOVING and begins rotation.
     */
    @Override
    public void initialize() {
        // Mark turret as actively moving
        turret.state = TurretState.MOVING;

        // Start rotating in the appropriate direction
        startRotation();
    }

    /**
     * Called repeatedly while command is running.
     * Continues rotation until target touch sensor is pressed.
     */
    @Override
    public void execute() {
        robot.sensors.addTelemetry("Turret rotate pwr", String.valueOf(turret.spinServo.getPower()));
    }

    /**
     * Determines when the command should finish.
     * Command finishes when the target touch sensor is triggered.
     *
     * @return true when target position is reached
     */
    @Override
    public boolean isFinished() {
        // Check which sensor should stop the rotation
        if (targetState == TurretState.FRONT) {
            return turret.frontButton.isPressed();
        } else if (targetState == TurretState.LEFT) {
            return turret.leftButton.isPressed();
        }
        // Safety: shouldn't reach here, but stop if target is MOVING
        return false;
    }

    /**
     * Called once when command ends (either finishes or is interrupted).
     * Stops servo and updates turret state to target position.
     *
     * @param interrupted true if command was interrupted, false if finished normally
     */
    @Override
    public void end(boolean interrupted) {
        // Stop the servo
        turret.spinServo.setPower(0);

        // Update state to target position (unless interrupted)
        if (!interrupted) {
            turret.state = targetState;
        }
    }

    /**
     * Determines rotation direction and starts the servo.
     * Direction depends on current and target positions.
     */
    private void startRotation() {
        // Determine which direction to rotate
        double power = 0;

        if (targetState == TurretState.FRONT) {
            // Rotate toward front position
            // Adjust sign (+/-) based on your mechanism's direction
            power = Constants.TURRET_ROTATION_POWER;
        } else if (targetState == TurretState.LEFT) {
            // Rotate toward left position
            // Adjust sign (+/-) based on your mechanism's direction
            power = -Constants.TURRET_ROTATION_POWER;
        }

        turret.spinServo.setPower(power);
    }

}