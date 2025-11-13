package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Ganymede;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;

public class LauncherLaunch extends CommandBase {
    private Ganymede robot;
    private Launcher launcher;

    private boolean ballOutputted = false;

    public LauncherLaunch(Ganymede robot) {
        this.robot = robot;
        this.launcher = robot.launcher;

        addRequirements(launcher);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    // TODO: FIx threshold and reset variables
    @Override
    public void execute() {
        launcher.launcher.setPower(0.9);

        if (launcher.launcher.getVelocity() > 1700) {
            launcher.stopper.setPower(1);
        }

        if (launcher.launcher.getVelocity() < 1950 && launcher.launcher.getVelocity() > 1900) {
            launcher.stopper.setPower(0);
            ballOutputted = true;
        }
    }

    @Override
    public boolean isFinished() {
        return ballOutputted;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);

        launcher.launcher.setPower(0);
    }
}
