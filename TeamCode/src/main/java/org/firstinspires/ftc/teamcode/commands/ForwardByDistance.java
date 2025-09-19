package org.firstinspires.ftc.teamcode.commands;


import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.Ganymede;
import org.firstinspires.ftc.teamcode.subsystems.PedroDrive;

import java.util.concurrent.TimeUnit;

public class ForwardByDistance extends CommandBase {
    // REFERENCES
    private Ganymede robot;
    private PedroDrive drive;
    private double distance; // in inches ??????????
    protected boolean finished = false;


    protected Timing.Timer timer;

    public ForwardByDistance(Ganymede robot, double distance, double timeoutMilliseconds) {
        this.robot = robot;
        this.drive = robot.drive;
        this.distance = distance;
        timer = new Timing.Timer((long)timeoutMilliseconds, TimeUnit.MILLISECONDS);

        addRequirements(drive);
    }

    @Override
    public void execute() {
        super.execute();




    }

    @Override
    public void initialize() {
        timer.start();


    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return finished || timer.done();
    }
}
