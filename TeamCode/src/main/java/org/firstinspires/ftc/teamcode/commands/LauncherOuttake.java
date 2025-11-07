package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Ganymede;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;

public class LauncherOuttake extends CommandBase {
    private Ganymede robot;
    private Launcher launcher;

    public LauncherOuttake(Ganymede robot) {
        this.robot = robot;
        this.launcher = robot.launcher;

        addRequirements(launcher);
    }

    @Override
    public void execute() {
        launcher.launcher.setPower(0.9);
        robot.sensors.addTelemetry("Motor Speed", String.valueOf(launcher.launcher.getVelocity()));
    }

    @Override
    public void end(boolean interrupted) {
        launcher.launcher.setPower(0);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
