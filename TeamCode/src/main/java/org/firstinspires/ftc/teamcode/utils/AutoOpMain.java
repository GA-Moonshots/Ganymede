package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Ganymede;
import org.firstinspires.ftc.teamcode.commands.SavePoseCommand;

/**
 * Main autonomous OpMode with simple alliance and position selection
 *
 * During init:
 * - Press X for BLUE, B for RED
 * - Press A for LEFT, Y for RIGHT
 * - Current selection shown on telemetry
 * - Press START to begin
 */
@Autonomous(name = "Autonomous - Main", group = "Competition")
public class AutoOpMain extends CommandOpMode {

    private boolean isRed;
    private boolean isNearGoal;
    private Ganymede robot;

    /**
     * Initialize and handle alliance/position selection
     */
    @Override
    public void initialize() {
        // Default starting configuration
        isRed = true;
        isNearGoal = true;

        // Selection loop - runs until START is pressed
        while (opModeInInit()) {
            // Alliance selection: X for blue, B for red
            if (gamepad1.x || gamepad2.x) {
                isRed = false;
            } else if (gamepad1.b || gamepad2.b) {
                isRed = true;
            }

            // Position selection: A for left, Y for right
            if (gamepad1.a || gamepad2.a) {
                isNearGoal = true;
            } else if (gamepad1.y || gamepad2.y) {
                isNearGoal = false;
            }

            // Display current selection
            telemetry.addLine("═══════════════════════════════");
            telemetry.addLine("   AUTONOMOUS CONFIGURATION");
            telemetry.addLine("═══════════════════════════════");
            telemetry.addLine();
            telemetry.addData("Alliance", isRed ? "RED (B)" : "BLUE (X)");
            telemetry.addData("Position", isNearGoal ? "Near Goal (A)" : "Far Goal (Y)");
            telemetry.addLine();
            telemetry.addLine("Press START when ready!");
            telemetry.update();
        }

        // Build robot with selected configuration
        robot = new Ganymede(this, isRed, isNearGoal);

        schedule(new SavePoseCommand(robot));
    }

    /**
     * Main autonomous execution loop
     */
    @Override
    public void run() {
        super.run();
    }

    /**
     * Cleanup when OpMode stops
     * SAVES BOTH POSE AND ALLIANCE COLOR for TeleOp
     */
    @Override
    public void reset() {
        super.reset();

        // Save final pose AND alliance color for TeleOp
        if (robot != null && robot.drive != null) {
            Pose finalPose = robot.drive.getPose();
            PersistentPoseManager.savePose(finalPose, isRed);
        }
    }
}