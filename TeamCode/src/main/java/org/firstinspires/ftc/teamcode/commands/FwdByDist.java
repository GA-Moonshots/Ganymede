package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.Ganymede;
import org.firstinspires.ftc.teamcode.utils.Constants;

/**
 * Command to move the robot forward or backward by a specified distance.
 *
 * Uses the robot's current heading to calculate target position, then
 * follows a straight-line path to reach it. Positive distance = forward,
 * negative distance = backward.
 *
 * Typical timeout: 3-5 seconds for short moves, 5-10s for longer distances
 */
public class FwdByDist extends DriveAbstract {

    // ============================================================
    //                     COMMAND STATE
    // ============================================================

    private final double distance; // in inches (positive = FWD, negative = BACK)
    private Pose targetPose;
    private boolean finished = false;

    // ============================================================
    //                     CONSTRUCTOR
    // ============================================================

    /**
     * Creates a command to move forward/backward by distance.
     *
     * @param robot Main robot object
     * @param distance How far to move in inches (+ = forward, - = backward)
     * @param timeoutSeconds Safety timeout in seconds (typically 3-10s)
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

        Pose currentPose = follower.getPose();

        // Calculate target pose based on current heading and distance
        double currentHeading = currentPose.getHeading();
        double deltaX = distance * Math.cos(currentHeading);
        double deltaY = distance * Math.sin(currentHeading);

        targetPose = new Pose(
                currentPose.getX() + deltaX,
                currentPose.getY() + deltaY,
                currentHeading
        );

        // Create and follow straight-line path
        Path path = new Path(new BezierLine(currentPose, targetPose));
        follower.followPath(path);
    }

    @Override
    public void execute() {
        super.execute();

        // Check if we've reached the target pose within tolerance
        if (follower.atPose(targetPose, Constants.POSE_TOLERANCE, Constants.POSE_TOLERANCE)) {
            finished = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        standardCleanup();
    }

    @Override
    public boolean isFinished() {
        return finished || timer.done();
    }
}