package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Ganymede;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;

public class LauncherDefault extends CommandBase {
    private Ganymede robot;
    private Launcher launcher;

    private double defaultLauncherSpeed;

    public LauncherDefault(Ganymede robot) {
        this.robot = robot;
        this.launcher = this.robot.launcher;

        defaultLauncherSpeed = 0.3;

        addRequirements(launcher);
    }

    @Override
    public void execute() {
        launcher.launcher.setPower(defaultLauncherSpeed);
    }
}
