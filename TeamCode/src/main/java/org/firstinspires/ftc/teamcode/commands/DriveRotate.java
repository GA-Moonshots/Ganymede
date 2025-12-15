package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;

import org.firstinspires.ftc.teamcode.Ganymede;
import org.firstinspires.ftc.teamcode.utils.Constants;


public class DriveRotate extends DriveAbstract {

    // ============================================================
    //                     COMMAND STATE
    // ============================================================

    private final double rotation;
    private Pose targetPose;
    private boolean finished = false;

    // ============================================================
    //                     CONSTRUCTOR

    /**
     * Creates a command to rotate.
     *
     * @param robot Main robot object containing all subsystems
     * @param rotation degree to move (positive = clockwise, negative = counter clockwise) in inches
     * @param timeoutSeconds Safety timeout in seconds
     */
    public DriveRotate(Ganymede robot, double rotation, double timeoutSeconds) {
        super(robot, timeoutSeconds);
        this.rotation = rotation;
    }

    // ============================================================
    //                     COMMAND LIFECYCLE

    @Override
    public void initialize() {
        timer.start();

        Pose currentPose = drive.getPose();

        // Get normalized heading to handle ±180° wrap-around correctly
        double targetHeading = drive.getNormalizedHeading() + rotation;

        // Create target pose by adding forward vector to current position
        targetPose = new Pose(
                currentPose.getX(),
                currentPose.getY(),
                Math.toRadians(targetHeading)
        );

        PathBuilder builder = new PathBuilder(robot.drive.follower).addPath(new BezierCurve(currentPose, targetPose));
        follower.followPath(builder.build());
    }

    @Override
    public void execute() {
        super.execute();

        // Check if we've reached target within tolerance
        if (follower.atPose(targetPose, Constants.POSE_TOLERANCE, Constants.POSE_TOLERANCE)) {
            finished = true;
        }

    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        standardCleanup();
        drive.follower.breakFollowing();
    }

    @Override
    public boolean isFinished() {
        return finished || timer.done();
    }
}