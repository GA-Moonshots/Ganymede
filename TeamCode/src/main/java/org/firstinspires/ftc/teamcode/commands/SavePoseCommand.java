package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Ganymede;
import org.firstinspires.ftc.teamcode.subsystems.PedroDrive;
import org.firstinspires.ftc.teamcode.utils.TeleOpMain;
import org.firstinspires.ftc.teamcode.utils.PersistentPoseManager;

public class SavePoseCommand extends CommandBase {
    private final PedroDrive driveSubsystem;

    public SavePoseCommand(Ganymede robot) {
        this.driveSubsystem = robot.drive;
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        Pose finalPose = driveSubsystem.getPose();
        PersistentPoseManager.savePose(finalPose);
    }
}