package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.Ganymede;
import org.firstinspires.ftc.teamcode.utils.Constants;

/**
 * Move forward or backward by distance.
 * Uses PedroDrive angle translators to fix ±180° wrap-around bug.
 */
public class FwdByDist extends DriveAbstract {

    private final double distance;
    private Pose targetPose;
    private boolean finished = false;

    public FwdByDist(Ganymede robot, double distance, double timeoutSeconds) {
        super(robot, timeoutSeconds);
        this.distance = distance;
    }

    @Override
    public void initialize() {
        timer.start();

        Pose currentPose = drive.getPose();

        // Use normalized heading to fix wrap-around bug
        double currentHeading = drive.getNormalizedHeading();

        // Calculate forward vector
        double deltaX = distance * Math.cos(currentHeading);
        double deltaY = distance * Math.sin(currentHeading);

        targetPose = new Pose(
                currentPose.getX() + deltaX,
                currentPose.getY() + deltaY,
                drive.toPedroHeading(currentHeading)
        );

        // Create and follow path
        Path path = new Path(new BezierLine(currentPose, targetPose));
        follower.followPath(path);
    }

    @Override
    public void execute() {
        super.execute();

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