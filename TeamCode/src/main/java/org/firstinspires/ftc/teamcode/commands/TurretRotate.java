package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Ganymede;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

public class TurretRotate extends CommandBase {

    private Ganymede robot;
    private Turret turret;

    public TurretRotate(Ganymede robot) {
        this.robot = robot;
        this.turret = robot.turret;

        addRequirements(turret);
    }

    @Override
    public void execute() {}
}
