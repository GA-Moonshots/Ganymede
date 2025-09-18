package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Ganymede;
import org.firstinspires.ftc.teamcode.subsystems.PedroDrive;

public class ForwardByDistance extends CommandBase {
    private Ganymede robot;
    private PedroDrive drive;

    public ForwardByDistance(Ganymede robot) {
        this.robot = robot;
        this.drive = robot.drive;

        addRequirements(drive);
    }
}
