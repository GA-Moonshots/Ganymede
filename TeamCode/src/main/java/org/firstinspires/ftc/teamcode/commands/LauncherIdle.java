package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Ganymede;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.utils.Constants;

/**
 * Default command for the Launcher subsystem.
 * Keeps the flywheel spinning at a low idle power so that
 * spin-up time is shorter when a launch is triggered.
 * Runs continuously whenever no other launcher command is active.
 */
public class LauncherIdle extends CommandBase {
    private Ganymede robot;
    private Launcher launcher;

    public LauncherIdle(Ganymede robot) {
        this.robot = robot;
        this.launcher = this.robot.launcher;

        addRequirements(launcher);
    }

    @Override
    public void execute() {
        launcher.launcher.setPower(Constants.LAUNCHER_IDLE_POWER);
    }
}
