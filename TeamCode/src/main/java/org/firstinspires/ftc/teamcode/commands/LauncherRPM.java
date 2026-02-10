package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.hardware.CRServo;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Ganymede;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.Constants;

import java.util.concurrent.TimeUnit;

/**
 * RPM-based launcher command.
 * Spins the flywheel up to launch power, waits until RPM reaches the feed
 * threshold, feeds the ball, then detects the launch by the RPM drop.
 */
public class LauncherRPM extends CommandBase {
    private Ganymede robot;
    private Launcher launcher;
    private CRServo feeder;
    private Timing.Timer timer;

    int runFeeder;
    double launcherSpeed;

    private boolean ballOutputted;
    private boolean readyToLaunch;

    public LauncherRPM(Ganymede robot, double launcherSpeed) {
        this.robot = robot;
        this.launcher = this.robot.launcher;
        this.launcherSpeed = launcherSpeed;
        this.timer = new Timing.Timer(
                (long)(Constants.LAUNCHER_RPM_TIMEOUT_SECONDS * 1000), TimeUnit.MILLISECONDS);

        addRequirements(launcher);
    }

    public LauncherRPM(Ganymede robot) {
        this.robot = robot;
        this.launcher = this.robot.launcher;
        this.launcherSpeed = Constants.LAUNCHER_DEFAULT_POWER;
        this.timer = new Timing.Timer(
                (long)(Constants.LAUNCHER_RPM_TIMEOUT_SECONDS * 1000), TimeUnit.MILLISECONDS);

        addRequirements(launcher);
    }

    @Override
    public void initialize() {
        super.initialize();
        ballOutputted = false;
        readyToLaunch = false;

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
        launcher.launcher.setPower(launcherSpeed);

        // Conversion from deg/s to rpm
        double currentSpeed = launcher.launcher.getVelocity(AngleUnit.DEGREES) / 6;
        robot.sensors.addTelemetry("Current RPM: ", String.valueOf(currentSpeed));

        double activeCurrent = launcher.launcher.getCurrent(CurrentUnit.AMPS);
        robot.sensors.addTelemetry("Motor Current (A): ", String.valueOf(activeCurrent));

        // Start feeding once flywheel reaches target RPM
        if(currentSpeed >= Constants.LAUNCHER_FEED_RPM) {
            feeder.setPower(runFeeder);
            readyToLaunch = true;
        }

        // RPM drop after feeding indicates ball has been launched
        if (currentSpeed <= Constants.LAUNCHER_LAUNCHED_RPM && readyToLaunch) {
            feeder.setPower(0);
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
