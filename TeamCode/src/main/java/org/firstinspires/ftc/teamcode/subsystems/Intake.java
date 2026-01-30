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
        robot.sensors.addTelemetry("isGreen", String.valueOf(robot.sensors.isGreen()));

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

}