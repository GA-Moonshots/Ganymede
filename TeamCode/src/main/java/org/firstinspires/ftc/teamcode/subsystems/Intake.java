package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Ganymede;
import org.firstinspires.ftc.teamcode.utils.Constants;

/**
 * ╔═══════════════════════════════════════════════════════════════════════════╗
 * ║                          INTAKE SUBSYSTEM                                 ║
 * ║                                                                           ║
 * ║  Controls the intake mechanism for collecting game elements.              ║
 * ║  Uses a single motor for intake/outtake operations.                       ║
 * ╚═══════════════════════════════════════════════════════════════════════════╝
 */
public class Intake extends SubsystemBase {

    // ============================================================
    //                     SUBSYSTEM COMPONENTS

    private Ganymede robot;

    /** Main intake motor */
    public DcMotorEx intakeMotor;

    public Servo stopperG;
    public Servo stopperP;
    public Servo linearServo;
    // ============================================================
    //                        CONSTRUCTOR

    /**
     * Initializes the intake subsystem and loads hardware from config.
     *
     * @param robot Main robot instance containing hardware map
     */
    public Intake(Ganymede robot) {
        this.robot = robot;

        // Load motor from hardware map
        this.intakeMotor = robot.hardwareMap.get(DcMotorEx.class, Constants.INTAKE_NAME);

        this.stopperG = robot.hardwareMap.get(Servo.class, Constants.GREEN_SORTER_NAME);
        this.stopperP = robot.hardwareMap.get(Servo.class, Constants.PURPLE_SORTER_NAME);
        this.linearServo = robot.hardwareMap.get(Servo.class, Constants.LINEAR_SERVO);


        stopperG.setPosition(0.5);
        stopperP.setPosition(0);
    }

    // ============================================================
    //                     SUBSYSTEM METHODS

    /**
     * Stops the intake motor.
     * Called when subsystem needs to halt all movement.
     */

    @Override

    public void periodic(){
        if (robot.sensors.isGreen()) {
            stopperG.setPosition(0);
            stopperP.setPosition(0.5);
        } else if (robot.sensors.isPurple()) {
            stopperG.setPosition(0.5);
            stopperP.setPosition(0);
        }

    }
    public void stop() {
        intakeMotor.setPower(0);
    }

    double CLOSED_POS = 0.2;
    double OPEN_POS = 0.8;
    
    public void extendServo() {
        double current = linearServo.getPosition();

        // Check which side it's closer to
        if (Math.abs(current - CLOSED_POS) < 0.05) {
            // It's closed → open it
            linearServo.setPosition(OPEN_POS);
        } else {
            // It's open (or somewhere else) → close it
            linearServo.setPosition(CLOSED_POS);
        }

    }

}