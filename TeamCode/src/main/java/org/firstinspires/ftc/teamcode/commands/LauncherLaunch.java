package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.hardware.CRServo;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.Ganymede;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import java.util.concurrent.TimeUnit;

public class LauncherLaunch extends CommandBase {
    private Ganymede robot;
    private Launcher launcher;
    private CRServo feeder; // we'll use this for green or purple
    int runFeeder;
    double launcherSpeed;

    private Timing.Timer timer;

    private boolean ballOutputted = false;

    public LauncherLaunch(Ganymede robot) {
        this.robot = robot;
        this.launcher = robot.launcher;
        launcherSpeed = 0.7; //0.85

        this.timer = new Timing.Timer(15, TimeUnit.SECONDS);

        addRequirements(launcher);
    }

    public LauncherLaunch(Ganymede robot, double launcherSpeed) {
        this.robot = robot;
        this.launcher = robot.launcher;
        this.launcherSpeed = launcherSpeed;

        this.timer = new Timing.Timer(15, TimeUnit.SECONDS);

        addRequirements(launcher);
    }

    @Override
    public void initialize() {
        super.initialize();
        ballOutputted = false;

        // determine the correct feeder
        if(robot.turret.state == Turret.TurretState.LEFT) {
            feeder = robot.launcher.greenFeeder;
            runFeeder = 1; // to be reversed because of gearing
        }
        else {
            feeder = robot.launcher.purpleFeeder;
            runFeeder = 1; // reversed because of gearing
        }

        timer.start();
    }

    @Override
    public void execute() {
        launcher.launcher.setPower(launcherSpeed); // LAUNCHER POWER
        robot.sensors.addTelemetry("Motor Speed", String.valueOf(launcher.launcher.getVelocity()));

        // START FEEDING
        if (timer.elapsedTime() >= 2) {
            feeder.setPower(runFeeder);
        }
        // GREEN TIMING
        if (robot.turret.state == Turret.TurretState.LEFT) {
            // STOP FEEDING
            if (timer.elapsedTime() >= 5) { //used to be 4
                feeder.setPower(0);
            }

            // GIVE TIME FOR BALL TO GET THROUGH
            if (timer.elapsedTime() >= 6) {
                ballOutputted = true;
            }
        }
        // PURPLE TIMING
        else {
            // STOP FEEDING
            if (timer.elapsedTime() >= 3.33) { // used to be 4
                feeder.setPower(0);
            }

            // GIVE TIME FOR BALL TO GET THROUGH
            if (timer.elapsedTime() >= 3.83) {
                ballOutputted = true;
            }
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
