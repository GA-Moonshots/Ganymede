package org.firstinspires.ftc.teamcode.utils;

import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Ganymede;

/**
 * Main TeleOp mode using CommandOpMode
 *
 * CommandOpMode automatically handles:
 * - Command scheduling and execution
 * - Subsystem periodic updates
 * - Gamepad button state updates
 *
 * This replaces the old DriveyMcDriverson
 */
@TeleOp(name = "TeleOp - Main", group = "Competition")
public class TeleOpMain extends CommandOpMode {

    private Ganymede robot;

    @Override
    public void initialize() {
        // Create the robot - this initializes all subsystems
        robot = new Ganymede(this);

        // The command scheduler is now active and will:
        // 1. Run the default Drive command continuously
        // 2. Handle all button bindings
        // 3. Call periodic() on all subsystems each loop

        telemetry.addData("Status", "Ready to run!");
        telemetry.update();
    }

    // That's it! No run() method needed!
    // CommandOpMode handles the main loop automatically

    // Optional: Override these if you need them

    @Override
    public void run() {
        super.run(); // This runs the command scheduler

        // Add any additional per-loop code here if needed
        // But usually you don't need anything here
    }

    @Override
    public void reset() {
        super.reset();
        // Called when the OpMode is stopped
        // Good place for cleanup if needed
    }
}