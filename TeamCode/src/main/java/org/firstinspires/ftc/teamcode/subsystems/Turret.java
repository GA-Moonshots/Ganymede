package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Ganymede;
import org.firstinspires.ftc.teamcode.utils.Constants;

public class Turret extends SubsystemBase {

    // ============================================================
    //                     TURRET STATE ENUM
    // ============================================================
    /**
     * Tracks the current position/state of the turret mechanism.
     * FRONT: Turret aligned to front position (frontButton pressed)
     * LEFT: Turret aligned to left position (leftButton pressed)
     * MOVING: Turret actively rotating between positions
     */
    public enum TurretState {
        FRONT,
        LEFT,
        MOVING
    }

    // ============================================================
    //                     SUBSYSTEM COMPONENTS
    // ============================================================

    private Ganymede robot;

    // Sensors unique to this subsystem
    public TouchSensor frontButton;
    public TouchSensor leftButton;

    // Servos
//    public CRServo spinServo;
    public Servo spinServo;

    // State tracking - public for direct access from commands
    public TurretState state = TurretState.FRONT ;  // Default to FRONT position

    // ============================================================
    //                        CONSTRUCTOR
    // ============================================================

    public Turret(Ganymede robot){
        this.robot = robot;

        // Load motors from hardware map using Constants
        spinServo = robot.hardwareMap.get(Servo.class, Constants.TURRET_SERVO_NAME);
//        frontButton = robot.hardwareMap.get(TouchSensor.class, Constants.FRONT_BUTTON_NAME);
//        leftButton = robot.hardwareMap.get(TouchSensor.class, Constants.LEFT_BUTTON_NAME);
    }

    // ============================================================
    //                     PERIODIC TELEMETRY
    // ============================================================

    @Override
    public void periodic(){
        robot.sensors.addTelemetry("Turret State", state.toString());
//        robot.sensors.addTelemetry("Left Button", String.valueOf(leftButton.isPressed()));
//        robot.sensors.addTelemetry("Front Button", String.valueOf(frontButton.isPressed()));
    }

}