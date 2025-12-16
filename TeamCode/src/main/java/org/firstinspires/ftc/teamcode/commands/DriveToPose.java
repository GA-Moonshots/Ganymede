package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.Ganymede;
import org.firstinspires.ftc.teamcode.utils.Constants;

public class DriveToPose extends DriveAbstract {
    // ============================================================
    //                     COMMAND STATE
    // ============================================================

    private Pose targetPose;
    private boolean finished = false;

    // ============================================================
    //                     CONSTRUCTOR

    /**
     * Creates a command to move forward/backward by distance.
     *
     * @param robot Main robot object containing all subsystems
     * @param timeoutSeconds Safety timeout in seconds
     */
    public DriveToPose(Ganymede robot, Pose target, double timeoutSeconds) {
        super(robot, timeoutSeconds);
        this.targetPose = target;
    }

    // ============================================================
    //                     COMMAND LIFECYCLE

    @Override
    public void initialize() {
        timer.start();

        Pose currentPose = drive.getPose();

        // Get normalized heading to handle ±180° wrap-around correctly
        double currentHeading = drive.getNormalizedHeading();

        PathBuilder builder = new PathBuilder(robot.drive.follower)
                .addPath(new BezierCurve(currentPose, targetPose))
                .setConstantHeadingInterpolation(targetPose.getHeading());
        follower.followPath(builder.build());

        // Telemetry for debugging
        robot.telemetry.addData("FwdByDist", "Initialized");
        robot.telemetry.addData("Current Heading", "%.1f°", Math.toDegrees(currentHeading));
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
        drive.follower.breakFollowing();

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
