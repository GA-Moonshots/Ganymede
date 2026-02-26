package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Ganymede;
import org.firstinspires.ftc.teamcode.utils.Constants;

public class Turret extends SubsystemBase {

    // ============================================================
    //                     POSITION CONSTANTS
    // ============================================================

    private static final double TURRET_FRONT_POSITION = 0.0; // change values ??
    private static final double TURRET_LEFT_POSITION  = 0.6;

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

    // Servo
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

    }

    // ============================================================
    //                     PERIODIC TELEMETRY
    // ============================================================

    @Override
    public void periodic(){
//        robot.sensors.addTelemetry("═══ Turret ═══", "");
//        robot.sensors.addTelemetry("Turret State", state.toString());
//        robot.sensors.addTelemetry("Spin Servo Positio,", String.valueOf(spinServo.getPosition()));
    }

    // ============================================================
    //                      POSITION HELPERS
    // ============================================================

    public void rotateToFront() {
        spinServo.setPosition(TURRET_FRONT_POSITION);
        state = TurretState.FRONT;
    }

    public void rotateToLeft() {
        spinServo.setPosition(TURRET_LEFT_POSITION);
        state = TurretState.LEFT;
    }

}