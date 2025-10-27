package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.Ganymede;
import org.firstinspires.ftc.teamcode.subsystems.PedroDrive;
import org.firstinspires.ftc.teamcode.utils.Constants;

import java.util.concurrent.TimeUnit;

public class DriveToBlue extends CommandBase {
    private final Ganymede robot;
    private final PedroDrive drive;
    private final Follower follower;

    public Pose targetPose;
    public boolean finished = false;

    protected Timing.Timer timer;

    public DriveToBlue(Ganymede robot, double timeoutMilliseconds) {
        this.robot = robot;
        this.drive = robot.drive;

        timer = new Timing.Timer((long)timeoutMilliseconds, TimeUnit.MILLISECONDS);
        follower = drive.follower;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        timer.start();

        Pose currentPose = follower.getPose();
        targetPose = new Pose(105, 32, drive.getPose().getHeading());

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
