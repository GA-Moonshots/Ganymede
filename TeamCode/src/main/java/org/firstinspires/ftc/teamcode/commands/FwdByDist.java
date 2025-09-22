package org.firstinspires.ftc.teamcode.commands;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.Ganymede;
import org.firstinspires.ftc.teamcode.subsystems.PedroDrive;
import org.firstinspires.ftc.teamcode.utils.Constants;

import java.util.concurrent.TimeUnit;

public class FwdByDist extends CommandBase {
    // REFERENCES
    private Ganymede robot;
    private PedroDrive drive;
    private double distance; // in inches positive = FWD, negative = BACK
    private Pose targetPose;
    private Follower follower;

    protected boolean finished = false;
    protected Timing.Timer timer;

    public FwdByDist(Ganymede robot, double distance, double timeoutMilliseconds) {
        this.robot = robot;
        this.drive = robot.drive;
        this.distance = distance;
        timer = new Timing.Timer((long)timeoutMilliseconds, TimeUnit.MILLISECONDS);

        follower = drive.follower;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        timer.start();

        Pose currentPose = follower.getPose();

        // calculate what our desired pose is given the distance
        double currentHeading = currentPose.getHeading();
        double deltaX = distance * Math.cos(currentHeading);
        double deltaY = distance * Math.sin(currentHeading);
        targetPose = new Pose(
                currentPose.getX() + deltaX,
                currentPose.getY() + deltaY,
                currentHeading
        );

        Path path = new Path(new BezierLine(currentPose, targetPose));

        follower.followPath(path);
    }

    @Override
    public void execute() {
        super.execute();
        if(follower.atPose(targetPose, Constants.POSE_TOLERANCE, Constants.POSE_TOLERANCE)){
            finished = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        follower.breakFollowing();
        drive.stop();
    }

    @Override
    public boolean isFinished() {
        return finished || timer.done();
    }
}
