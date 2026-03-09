package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Ganymede;
import org.firstinspires.ftc.teamcode.commands.LauncherRPM;

/**
 * ════════════════════════════════════════════════════════
 *                   LAUNCHER TUNER
 * ════════════════════════════════════════════════════════
 *
 * Standalone TeleOp for filling in LAUNCHER_RPM_TABLE.
 * Drive the robot to each target distance, dial in the
 * RPM that lands the ball, and record the (dist, RPM) pair.
 *
 * Controls (Player 1):
 *   START            — toggle alliance (RED ↔ BLUE)
 *   Left Bumper      — testRPM + 5
 *   D-Pad Down       — testRPM - 5
 *   Y                — fire one shot at testRPM
 *   Left stick       — drive/strafe (same as normal TeleOp)
 *   Right stick      — rotate
 *
 * After collecting all four (distance, RPM) pairs, update
 * Constants.LAUNCHER_RPM_TABLE in Constants.java.
 */
@TeleOp(name = "Launcher Tuner", group = "Utils")
public class LauncherTuner extends CommandOpMode {

    private Ganymede robot;

    private int testRPM = (int) Constants.LAUNCHER_FEED_RPM; // start at 150

    // Rising-edge tracking
    private boolean lastStart  = false;
    private boolean lastLB     = false;
    private boolean lastDpadDn = false;
    private boolean lastY      = false;

    @Override
    public void initialize() {
        robot = new Ganymede(this);
        telemetry.addData("Status", "Launcher Tuner ready  |  Y = fire");
        telemetry.update();
    }

    @Override
    public void run() {
        robot.player1.readButtons();
        robot.player2.readButtons();
        super.run(); // runs the command scheduler (Drive default command, etc.)

        boolean start  = gamepad1.start;
        boolean lb     = gamepad1.left_bumper;
        boolean dpadDn = gamepad1.dpad_down;
        boolean y      = gamepad1.y;

        // Alliance toggle (rising edge)
        if (start && !lastStart) robot.isRed = !robot.isRed;

        // RPM adjustment (rising edge — press repeatedly to step)
        if (lb     && !lastLB)     testRPM = Math.min(testRPM + 5, 200);
        if (dpadDn && !lastDpadDn) testRPM = Math.max(testRPM - 5, 100);

        // Fire one shot at testRPM (rising edge)
        if (y && !lastY) new LauncherRPM(robot, testRPM).schedule();

        lastStart  = start;
        lastLB     = lb;
        lastDpadDn = dpadDn;
        lastY      = y;

        // ── Telemetry ──────────────────────────────────────────
        double dist        = robot.drive.getDistanceToGoal();
        double actualRPM   = robot.launcher.launcher.getVelocity(AngleUnit.DEGREES) / 6.0;
        double interpRPM   = Constants.distanceToRPM(dist);

        telemetry.addData("Alliance",          robot.isRed ? "RED  (START to toggle)" : "BLUE (START to toggle)");
        telemetry.addData("Distance to Goal",  "%.1f in", dist);
        telemetry.addData("Test RPM  [LB +5 | ↓ -5]", testRPM);
        telemetry.addData("Motor actual RPM",  "%.1f", actualRPM);
        telemetry.addData("interpRPM @ dist",  "%.1f  ← table would pick", interpRPM);
    }
}
