package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;

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
 * ║  For heading θ, the forward vector components are:                        ║
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

        // Get normalized heading to handle ±180° wrap-around correctly
        double currentHeading = drive.getNormalizedHeading();

        // CRITICAL: Calculate forward vector components
        // cos(θ) gives X component, sin(θ) gives Y component
        // DO NOT swap these or robot will drive sideways!
        double deltaX = distance * Math.cos(currentPose.getHeading());  // ← Horizontal
        double deltaY = distance * Math.sin(currentPose.getHeading());  // ← Vertical

        // Create target pose by adding forward vector to current position
        targetPose = new Pose(
                currentPose.getX() + deltaX,
                currentPose.getY() + deltaY,
                currentPose.getHeading()  // Maintain heading
        );

        // Create and follow straight-line path
//        Path path = new Path(new BezierLine(currentPose, targetPose));
        PathBuilder builder = new PathBuilder(robot.drive.follower).addPath(new BezierCurve(currentPose, targetPose));
        follower.followPath(builder.build());

        // Telemetry for debugging
        robot.telemetry.addData("FwdByDist", "Initialized");
        robot.telemetry.addData("Current Heading", "%.1f°", Math.toDegrees(currentHeading));
        robot.telemetry.addData("Distance", "%.1f inches", distance);
        robot.telemetry.addData("Current Pos", "(%.1f, %.1f)",
                currentPose.getX(), currentPose.getY());
        robot.telemetry.addData("Target Pos", "(%.1f, %.1f)",
                targetPose.getX(), targetPose.getY());
        robot.telemetry.addData("Delta", "ΔX=%.1f, ΔY=%.1f", deltaX, deltaY);
    }

    @Override
    public void execute() {
        super.execute();

        // Check if we've reached target within tolerance
        if (follower.atPose(targetPose, Constants.POSE_TOLERANCE, Constants.POSE_TOLERANCE)) {
            finished = true;
        }

        // Update telemetry
        Pose current = drive.getPose();
        double distanceRemaining = Math.hypot(
                targetPose.getX() - current.getX(),
                targetPose.getY() - current.getY()
        );

        robot.telemetry.addData("Distance Remaining", "%.1f inches", distanceRemaining);
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