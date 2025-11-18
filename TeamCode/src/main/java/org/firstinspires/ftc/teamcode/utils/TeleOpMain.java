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

        // The command scheduler is now active and will:
        // 1. Run the default Drive command continuously
        // 2. Handle all button bindings
        // 3. Call periodic() on all subsystems each loop

        telemetry.addData("Status", "Ready to run!");
        telemetry.update();

        // if a is pressed, we are red
        if(robot.player1.gamepad.a) {
            robot.startPose = new Pose(95.65, -66.93, 0);
            // if b is pressed, we are blue
        } else if (robot.player1.gamepad.b) {
            robot.startPose = new Pose(132.07, -64.38, 0);
        }


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