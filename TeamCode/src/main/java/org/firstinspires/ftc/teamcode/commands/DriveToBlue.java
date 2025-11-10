package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.Ganymede;
import org.firstinspires.ftc.teamcode.utils.Constants;

/**
 * Command to drive to the blue alliance sample collection area.
 *
 * Drives to a specific field coordinate (105, 32) while maintaining
 * current heading. This is a hardcoded target position for autonomous.
 *
 * Typical timeout: 5-10 seconds depending on starting position
 */
public class DriveToBlue extends DriveAbstract {

    private Pose targetPose;
    private boolean finished = false;

    // ============================================================
    //                     CONSTRUCTOR

    /**
     * Creates a command to drive to the blue collection zone.
     *
     * @param robot Main robot object
     * @param timeoutSeconds Safety timeout in seconds (typically 5-10s)
     */
    public DriveToBlue(Ganymede robot, double timeoutSeconds) {
        super(robot, timeoutSeconds);
    }

    // ============================================================
    //                     COMMAND LIFECYCLE

    @Override
    public void initialize() {
        timer.start();

        Pose currentPose = follower.getPose();

        // Target: Blue sample collection area (X=105, Y=32)
        // Maintain current heading to avoid unnecessary rotation
        targetPose = new Pose(105, 32, currentPose.getHeading());

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