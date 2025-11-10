package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.Ganymede;
import org.firstinspires.ftc.teamcode.utils.Constants;

/**
 * ╔═══════════════════════════════════════════════════════════════════════════╗
 * ║                      FORWARD BY DISTANCE COMMAND                          ║
 * ║                                                                           ║
 * ║  Moves the robot forward or backward by a specified distance while         ║
 * ║  maintaining its current heading.                                         ║
 * ║                                                                           ║
 * ║  COORDINATE SYSTEM (Pedro Pathing standard):                              ║
 * ║    • 0° (0 rad) = Robot facing RIGHT (toward blue alliance)               ║
 * ║    • 90° (π/2 rad) = Robot facing FORWARD (toward scoring area)           ║
 * ║    • 180° (±π rad) = Robot facing LEFT (toward red alliance)              ║
 * ║    • -90° (-π/2 rad) = Robot facing BACKWARD                              ║
 * ║                                                                           ║
 * ║  FIXED: For heading θ, the forward vector components are:                 ║
 * ║    • ΔX = distance × cos(θ)  (horizontal component)                       ║
 * ║    • ΔY = distance × sin(θ)  (vertical component)                         ║
 * ║                                                                           ║
 * ╚═══════════════════════════════════════════════════════════════════════════╝
 */
public class FwdByDist extends DriveAbstract {

    // ============================================================
    //                     COMMAND STATE
    // ============================================================

    private final double distance;
    private Pose targetPose;
    private boolean finished = false;

    // ============================================================
    //                     CONSTRUCTOR
    // ============================================================

    /**
     * Creates a command to move forward/backward by distance.
     *
     * @param robot Main robot object containing all subsystems
     * @param distance Distance to move (positive = forward, negative = backward) in inches
     * @param timeoutSeconds Safety timeout in seconds
     */
    public FwdByDist(Ganymede robot, double distance, double timeoutSeconds) {
        super(robot, timeoutSeconds);
        this.distance = distance;
    }

    // ============================================================
    //                     COMMAND LIFECYCLE
    // ============================================================

    @Override
    public void initialize() {
        timer.start();

        Pose currentPose = drive.getPose();

        // Get BOTH raw and normalized heading for debugging
        double rawHeading = currentPose.getHeading();
        double normalizedHeading = drive.getNormalizedHeading();

        // *** CRITICAL FIX: Use cos for X, sin for Y ***
        // Previous bug had these swapped, causing 90° rotation!
        double deltaX = distance * Math.cos(normalizedHeading);  // ← cos for X (horizontal)
        double deltaY = distance * Math.sin(normalizedHeading);  // ← sin for Y (vertical)

        // Create target pose by adding forward vector to current position
        targetPose = new Pose(
                currentPose.getX() + deltaX,
                currentPose.getY() + deltaY,
                drive.toPedroHeading(normalizedHeading)  // Maintain heading
        );

        // Create and follow straight-line path
        Path path = new Path(new BezierLine(currentPose, targetPose));
        follower.followPath(path);

        // *** ENHANCED DEBUGGING TELEMETRY ***
        robot.telemetry.addData("═══ FwdByDist Debug ═══", "");
        robot.telemetry.addData("Distance Cmd", "%.1f inches", distance);

        robot.telemetry.addData("─── Heading Values ───", "");
        robot.telemetry.addData("RAW Heading", "%.4f", rawHeading);
        robot.telemetry.addData("RAW as degrees", "%.1f°", Math.toDegrees(rawHeading));
        robot.telemetry.addData("NORMALIZED", "%.4f", normalizedHeading);
        robot.telemetry.addData("NORMALIZED °", "%.1f°", Math.toDegrees(normalizedHeading));

        robot.telemetry.addData("─── Vector Math ───", "");
        robot.telemetry.addData("cos(heading)", "%.4f", Math.cos(normalizedHeading));
        robot.telemetry.addData("sin(heading)", "%.4f", Math.sin(normalizedHeading));
        robot.telemetry.addData("Delta X", "%.2f inches", deltaX);
        robot.telemetry.addData("Delta Y", "%.2f inches", deltaY);

        robot.telemetry.addData("─── Positions ───", "");
        robot.telemetry.addData("Current Pos", "(%.1f, %.1f)",
                currentPose.getX(), currentPose.getY());
        robot.telemetry.addData("Target Pos", "(%.1f, %.1f)",
                targetPose.getX(), targetPose.getY());
    }

    @Override
    public void execute() {
        super.execute();

        // Check if we've reached target within tolerance
        if (follower.atPose(targetPose, Constants.POSE_TOLERANCE, Constants.POSE_TOLERANCE)) {
            finished = true;
        }

        // Update telemetry with distance remaining
        Pose current = drive.getPose();
        double distanceRemaining = Math.hypot(
                targetPose.getX() - current.getX(),
                targetPose.getY() - current.getY()
        );

        robot.telemetry.addData("Distance Remaining", "%.1f inches", distanceRemaining);
        robot.telemetry.addData("Current Pos", "(%.1f, %.1f)",
                current.getX(), current.getY());
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        standardCleanup();

        if (interrupted) {
            robot.telemetry.addData("FwdByDist", "INTERRUPTED");
        } else {
            robot.telemetry.addData("FwdByDist", "Complete!");
        }
    }

    @Override
    public boolean isFinished() {
        return finished || timer.done();
    }
}