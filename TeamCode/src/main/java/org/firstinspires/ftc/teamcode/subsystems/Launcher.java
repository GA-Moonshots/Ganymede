package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.Ganymede;
import org.firstinspires.ftc.teamcode.utils.Constants;

public class Launcher extends SubsystemBase {

    private Ganymede robot;
    // declare motors

    public DcMotorEx launcher;

    public Servo stopper;


    // CONSTRUCTOR
    public Launcher(Ganymede robot){
        this.robot = robot;
        // load motors from hardware map using Constants.MOTOR_NAME
        launcher  = robot.hardwareMap.get(DcMotorEx.class, Constants.LAUNCHER_NAME);
        stopper= robot.hardwareMap.get(Servo.class, Constants.STOPER_SERVO_NAME);

    }

    //get state of servo
    public double  stopperState(){
        double state = stopper.getPosition();
        return state;
    }

    // HAPPENS 30x per second
    @Override
    public void periodic() {
        robot.sensors.addTelemetry("Intake" , "Feeling SUPER WELL HOLYYYYY");

    }

    public boolean readyToLaunch(){
        if (launcher.getVelocity() > 1000) {
            return true;
        }

        return false;
    }

    public void stop() {
        // kill power to motors
        launcher.setPower(0);
    }
}



