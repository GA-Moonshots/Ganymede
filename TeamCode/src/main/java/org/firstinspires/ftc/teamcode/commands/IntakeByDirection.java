package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Ganymede;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class IntakeByDirection extends CommandBase {
    private Ganymede robot;
    private Intake intake;

    private boolean direction;

    public IntakeByDirection(Ganymede robot, boolean in) {
        this.robot = robot;
        this.intake = this.robot.intake;

        this.direction = in;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.intakeMotor.setMotorEnable();

        if(direction) {
            intake.intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        } else {
            intake.intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }

    @Override
    public void execute() {
        intake.intakeMotor.setPower(1);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        intake.intakeMotor.setMotorDisable();
        intake.intakeMotor.setPower(0);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
