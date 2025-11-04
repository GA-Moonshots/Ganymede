package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Ganymede;
import org.firstinspires.ftc.teamcode.utils.Constants;

public class Turret extends SubsystemBase {

    private Ganymede robot;
    // declare motors

    public CRServo spinServo;

    // CONSTRUCTOR
    public Turret(Ganymede robot){
        this.robot = robot;
        // load motors from hardware map using Constants.MOTOR_NAME
        spinServo  = robot.hardwareMap.get(CRServo.class, Constants.TURRET_SERVO_NAME);
    }







}
