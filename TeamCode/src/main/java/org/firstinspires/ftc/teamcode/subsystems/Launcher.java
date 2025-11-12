package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;
//import com.seattlesolvers.solverslib.hardware.motors.CRServo;


import org.firstinspires.ftc.teamcode.Ganymede;
import org.firstinspires.ftc.teamcode.utils.Constants;

public class Launcher extends SubsystemBase {

    private Ganymede robot;
    // declare motors
    public DcMotorEx launcher;
    public CRServo stopper;

    // CONSTRUCTOR
    public Launcher(Ganymede robot){
        this.robot = robot;
        // load motors from hardware map using Constants.MOTOR_NAME
        launcher  = robot.hardwareMap.get(DcMotorEx.class, Constants.LAUNCHER_NAME);
      //  stopper = robot.hardwareMap.get(CRServo.class, Constants.STOPER_SERVO_NAME);
        stopper = robot.hardwareMap.get(CRServo.class, Constants.STOPER_SERVO_NAME);
    }

    // HAPPENS 30x per second
    @Override
    public void periodic() {
        // robot.sensors.addTelemetry("Intake" , "Feeling SUPER WELL HOLYYYYY");
    }

    public boolean readyToLaunch(){
        // TODO: dynamically calculate required speed based on location
        return launcher.getVelocity() > 2000;
    }

    public void spinServo(){
        stopper.setPower(1);

    }

   /* public void stopperOpen() {
        stopper.setPosition(0.6);
    }
    public void stopperClosed() {
        stopper.setPosition(0.3);
    }*/

    public void stop() { launcher.setPower(0); }
}



