package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.command.SubsystemBase;
//import com.seattlesolvers.solverslib.hardware.motors.CRServo;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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

    @Override
    public void periodic() {
        double currentRPM = launcher.getVelocity(AngleUnit.DEGREES) / 6.0;
        robot.sensors.addTelemetry("═══ Launcher ═══", "");
        robot.sensors.addTelemetry("Flywheel RPM", String.format("%.0f", currentRPM));
    }

    /** Returns true if flywheel RPM is at or above the feed threshold */
    public boolean isAtLaunchRPM() {
        double currentRPM = launcher.getVelocity(AngleUnit.DEGREES) / 6.0;
        return currentRPM >= Constants.LAUNCHER_FEED_RPM;
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



