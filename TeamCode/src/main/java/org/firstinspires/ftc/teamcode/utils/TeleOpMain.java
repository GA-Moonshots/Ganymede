package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.geometry.Pose;
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

        telemetry.addData("Status", "Ready to run!");
        telemetry.update();

    }


    @Override
    public void run() {
        robot.player1.readButtons();
        robot.player2.readButtons();
        super.run();
    }

    @Override
    public void reset() {
        super.reset();
        // Called when the OpMode is stopped
        // Good place for cleanup if needed
    }
}