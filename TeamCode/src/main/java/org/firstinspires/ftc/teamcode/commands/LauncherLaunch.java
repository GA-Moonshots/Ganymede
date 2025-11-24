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

        this.timer = new Timing.Timer(15, TimeUnit.SECONDS);

        addRequirements(launcher);
    }

    @Override
    public void initialize() {
        super.initialize();
        ballOutputted = false;
        timer.start();
    }

    @Override
    public void execute() {

        launcher.launcher.setPower(0.9);
        robot.sensors.addTelemetry("Motor Speed", String.valueOf(launcher.launcher.getVelocity()));

        // START FEEDING
        if (timer.elapsedTime() >= 2.5) {
            launcher.feedGreen();
        }

        // STOP FEEDING
        if (timer.elapsedTime() >= 7) {
            launcher.stopFeedingGreen();
        }

        // GIVE TIME FOR BALL TO GET THROUGH
        if (timer.elapsedTime() >= 9) {
            ballOutputted = true;
        }
    }

    @Override
    public boolean isFinished() {
        return ballOutputted || timer.done();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        launcher.stopAll();
    }
}
