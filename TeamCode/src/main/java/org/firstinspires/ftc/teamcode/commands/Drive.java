package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Ganymede;
import org.firstinspires.ftc.teamcode.subsystems.PedroDrive;
import org.firstinspires.ftc.teamcode.utils.Constants;

public class Drive extends CommandBase {
    private final PedroDrive drive;
    private GamepadEx player1;
    private Ganymede robot;

    private final FtcDashboard dashboard;

    private double strafeSpeed;
    private double forwardSpeed;
    private double turnSpeed;

    public Drive(Ganymede robot) {
        // pulling off some handy references
        this.robot = robot;
        this.drive = robot.drive;
        player1 = robot.player1;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(robot.drive);
        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void initialize(){
        // Do you need to set up anything when the command gets first added to the scheduler?
    }

    @Override
    public void execute() {
        // Slow mode activated if right bumper or lift is up
        double speedMod = robot.opMode.gamepad1.right_bumper
                ? 0.25 : 1; // slow mode

        // INTERPRET CONTROLLER
        double forward = applyDeadZone(player1.getLeftY());
        double strafe = applyDeadZone(player1.getLeftX());
        double turn = applyDeadZone(-player1.getRightX());

        robot.telemetry.addData("Right Stick", "X: %.2f Y: %.2f", player1.getRightX(), player1.getRightY());

        // Drive the robot with adjusted inputs:
        drive.drive(forward * speedMod, strafe * speedMod, turn * speedMod);
    }

    public boolean isFinished() {
        // currently just a placeholder... the default
        return false;
    }

    /**
     * Helper function for applying dead zone
     */
    private double applyDeadZone(double input) {
        return Math.abs(input) <= Constants.INPUT_THRESHOLD ? 0.0d : input;
    }
}
