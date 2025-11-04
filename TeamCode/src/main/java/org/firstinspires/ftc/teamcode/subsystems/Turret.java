package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Ganymede;
import org.firstinspires.ftc.teamcode.utils.Constants;

public class Turret extends SubsystemBase {

    private Ganymede robot;
    // sensors unique to this subsystem
    public TouchSensor frontButton;
    public TouchSensor leftButton;
    
    // declare motors

    public CRServo spinServo;

    // CONSTRUCTOR
    public Turret(Ganymede robot){
        this.robot = robot;
        // load motors from hardware map using Constants.MOTOR_NAME
        spinServo  = robot.hardwareMap.get(CRServo.class, Constants.TURRET_SERVO_NAME);
        frontButton = robot.hardwareMap.get(TouchSensor.class, Constants.FRONT_BUTTON_NAME);
        leftButton = robot.hardwareMap.get(TouchSensor.class, Constants.LEFT_BUTTON_NAME);
    }

    @Override
    public void periodic(){
        robot.sensors.addTelemetry("Left Button", String.valueOf(leftButton.isPressed()));
        robot.sensors.addTelemetry("Front Button", String.valueOf(frontButton.isPressed()));
    }

}
