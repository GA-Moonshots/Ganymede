package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.utils.Constants.INTAKE_NAME;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Ganymede;

public class Intake extends SubsystemBase {
    private Ganymede robot;
    public DcMotorEx intakeMotor;

    public Intake(Ganymede robot) {
        this.robot = robot;
        this.intakeMotor = robot.hardwareMap.get(DcMotorEx.class, INTAKE_NAME);
    }

    @Override
    public void periodic() {
        super.periodic();
    }


}
