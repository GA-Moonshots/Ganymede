package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Ganymede;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;

public class LauncherRawPower extends CommandBase {
    private Ganymede robot;
    private Launcher launcher;

    public LauncherRawPower(Ganymede robot) {
        this.robot = robot;
        this.launcher = robot.launcher;

        addRequirements(launcher);
    }

    @Override
    public void execute() {
        launcher.launcher.setPower(1.0);
        robot.sensors.addTelemetry("Motor Speed", String.valueOf(launcher.launcher.getVelocity()));
    }

    @Override
    public void end(boolean interrupted) {
        launcher.launcher.setPower(0);
    }

}
