package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.hardware.CRServo;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Ganymede;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

public class LauncherRPM extends CommandBase {
    private Ganymede robot;
    private Launcher launcher;
    private CRServo feeder;

    int runFeeder;
    double launcherSpeed;

    // temporary
    int counter = 0;

    private boolean ballOutputted;
    private double rpmThresh = 1000;

    private double baseCurrent;
    private double currentThresh = 2;
    private boolean spikeDetected = false;

    public LauncherRPM(Ganymede robot, double launcherSpeed) {
        this.robot = robot;
        this.launcher = this.robot.launcher;

        this.launcherSpeed = launcherSpeed;

        addRequirements(launcher);
    }

    public LauncherRPM(Ganymede robot) {
        this.robot = robot;
        this.launcher = this.robot.launcher;

        this.launcherSpeed = 0.7;

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

//        timer.start(); maybe add timer giver
    }

    @Override
    public void execute() {
        launcher.launcher.setPower(launcherSpeed);

        // Conversion from deg/s to rpm
        double currentSpeed = launcher.launcher.getVelocity(AngleUnit.DEGREES)/6;
        robot.sensors.addTelemetry("Current RPM: ", String.valueOf(currentSpeed));

        // just a temporary thing to test for voltage
        double activeCurrent = launcher.launcher.getCurrent(CurrentUnit.AMPS);
        if (counter == 30) {
            robot.sensors.addTelemetry("Motor Voltage: ", String.valueOf(activeCurrent));
        }

        // starts feeding
        if(currentSpeed >= rpmThresh) {
            feeder.setPower(runFeeder);
            baseCurrent = activeCurrent;
        }

        // timing on both side shouldn't matter, both should be similar
        // prone to error tho
        if (activeCurrent >= baseCurrent + currentThresh) {
            spikeDetected = true;
            feeder.setPower(0);
        }

        // Checking if the spike is done, our end condition
        if (spikeDetected && activeCurrent < baseCurrent + 0.5) {
            spikeDetected = false;
            ballOutputted = true;
        }
        
        counter++;
    }

    @Override
    public boolean isFinished() {
        return ballOutputted;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        launcher.stopAll(); // Potential error here
    }
}
