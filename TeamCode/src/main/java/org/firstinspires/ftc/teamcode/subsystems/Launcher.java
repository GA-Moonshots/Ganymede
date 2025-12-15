package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.command.SubsystemBase;
//import com.seattlesolvers.solverslib.hardware.motors.CRServo;


import org.firstinspires.ftc.teamcode.Ganymede;
import org.firstinspires.ftc.teamcode.utils.Constants;

public class Launcher extends SubsystemBase {

    private Ganymede robot;
    // declare motors
    public DcMotorEx launcher;
    public CRServo greenFeeder;
    public CRServo purpleFeeder;


    // CONSTRUCTOR
    public Launcher(Ganymede robot){
        this.robot = robot;
        launcher  = robot.hardwareMap.get(DcMotorEx.class, Constants.LAUNCHER_NAME);
        greenFeeder = robot.hardwareMap.get(CRServo.class, Constants.GREEN_FEEDER_SERVO_NAME);
        purpleFeeder = robot.hardwareMap.get(CRServo.class, Constants.PURPLE_FEEDER_SERVO_NAME);
    }

    // HAPPENS 30x per second
    @Override
    public void periodic() {
        // robot.sensors.addTelemetry("Intake" , "Feeling SUPER");
    }

    public boolean readyToLaunch(){
        // TODO: dynamically calculate required speed based on location
        return launcher.getVelocity() > 2000;
    }

    public void feedGreen(){
        greenFeeder.setPower(1);
    }

    public void stopFeedingGreen() {
        greenFeeder.setPower(0);
    }

    public void feedPurple(){
        purpleFeeder.setPower(-1);
    }

    public void stopFeedingPurple() {
        purpleFeeder.setPower(0);
    }

    public void stopAll() {
        launcher.setPower(0);
        greenFeeder.setPower(0);
        purpleFeeder.setPower(0);
    }
}



