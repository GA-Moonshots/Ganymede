package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.Ganymede;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;

import java.util.concurrent.TimeUnit;

public class LauncherLaunch extends CommandBase {
    private Ganymede robot;
    private Launcher launcher;

    private Timing.Timer timer;

    private boolean ballOutputted = false;

    public LauncherLaunch(Ganymede robot) {
        this.robot = robot;
        this.launcher = robot.launcher;

        // Convert seconds to milliseconds for the timer
        long timeoutMillis = (long)(3 * 1000);
        this.timer = new Timing.Timer(timeoutMillis, TimeUnit.SECONDS);

        addRequirements(launcher);
    }

    @Override
    public void initialize() {
        super.initialize();
        ballOutputted = false;
        timer.start();
    }

    // TODO: FIx threshold and reset variables
    @Override
    public void execute() {

        launcher.launcher.setPower(0.9);
        robot.sensors.addTelemetry("Motor Speed", String.valueOf(launcher.launcher.getVelocity()));
        // TODO: Use timer as well as checking for motor speeds
        if (timer.elapsedTime() >= 2.5) {
            launcher.stopper.setPower(1);
        }

        if (timer.elapsedTime() >= 7) {
            launcher.stopper.setPower(0);
        }

        if (timer.elapsedTime() >= 9) {
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
