package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
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
    // ============================================================

    private Ganymede robot;

    /** Main intake motor */
    public DcMotorEx intakeMotor;

    // ============================================================
    //                        CONSTRUCTOR
    // ============================================================

    /**
     * Initializes the intake subsystem and loads hardware from config.
     *
     * @param robot Main robot instance containing hardware map
     */
    public Intake(Ganymede robot) {
        this.robot = robot;

        // Load motor from hardware map
        this.intakeMotor = robot.hardwareMap.get(DcMotorEx.class, Constants.INTAKE_NAME);
    }

    // ============================================================
    //                     SUBSYSTEM METHODS
    // ============================================================

    /**
     * Stops the intake motor.
     * Called when subsystem needs to halt all movement.
     */
    public void stop() {
        intakeMotor.setPower(0);
    }

}