package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Ganymede;
import org.firstinspires.ftc.teamcode.commands.SavePoseCommand;

/**
 * Main autonomous OpMode with alliance, position, and MOTIF pattern selection
 *
 * During init:
 * - Press X for BLUE, B for RED
 * - Press A for LEFT, Y for RIGHT
 * - D-PAD LEFT for GPP, D-PAD DOWN for PGP, D-PAD RIGHT for PPG
 * - D-PAD UP to clear pattern (manual/no pattern following)
 * - Current selection shown on telemetry
 * - Press START to begin
 */
@Autonomous(name = "Autonomous - Main", group = "Competition")
public class AutoOpMain extends CommandOpMode {

    private boolean isRed;
    private boolean isNearGoal;
    private String motifPattern;
    private Ganymede robot;

    /**
     * Initialize and handle alliance/position/pattern selection
     */
    @Override
    public void initialize() {
        // Default starting configuration
        isRed = true;
        isNearGoal = true;
        motifPattern = "";  // Empty = no pattern selected yet

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

            // MOTIF Pattern selection: D-PAD
            // LEFT = GPP, DOWN = PGP, RIGHT = PPG, UP = clear
            if (gamepad1.dpad_left || gamepad2.dpad_left) {
                motifPattern = "GPP";
            } else if (gamepad1.dpad_down || gamepad2.dpad_down) {
                motifPattern = "PGP";
            } else if (gamepad1.dpad_right || gamepad2.dpad_right) {
                motifPattern = "PPG";
            } else if (gamepad1.dpad_up || gamepad2.dpad_up) {
                motifPattern = "";
            }

            // Display current selection
            telemetry.addLine("═══════════════════════════════");
            telemetry.addLine("   AUTONOMOUS CONFIGURATION");
            telemetry.addLine("═══════════════════════════════");
            telemetry.addLine();
            telemetry.addData("Alliance", isRed ? "RED (B)" : "BLUE (X)");
            telemetry.addData("Position", isNearGoal ? "Near Goal (A)" : "Far Goal (Y)");
            telemetry.addLine();
            telemetry.addLine("─────────── MOTIF ─────────────");
            telemetry.addData("Pattern", motifPattern.isEmpty() ? "NONE (D-PAD ↑)" : motifPattern);
            telemetry.addLine("  ← GPP   ↓ PGP   → PPG   ↑ Clear");
            telemetry.addLine();
            telemetry.addLine("Press START when ready!");
            telemetry.update();
        }

        // Build robot with selected configuration
        robot = new Ganymede(this, isRed, isNearGoal, motifPattern);

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