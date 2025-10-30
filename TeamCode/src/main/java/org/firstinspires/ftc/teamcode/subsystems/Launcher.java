package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.Ganymede;
import org.firstinspires.ftc.teamcode.utils.Constants;

public class Launcher extends SubsystemBase {

    private Ganymede robot;
    // declare motors

    public DcMotorEx launcher;

    // CONSTRUCTOR
    public Launcher(Ganymede robot){
        this.robot = robot;
        // load motors from hardware map using Constants.MOTOR_NAME
        launcher  = robot.hardwareMap.get(DcMotorEx.class, Constants.LAUNCHER_NAME);
    }

    // HAPPENS 30x per second
    @Override
    public void periodic() {
        robot.sensors.addTelemetry("Intake" , "Feeling SUPER WELL HOLYYYYY");
    }

    public void stop() {
        // kill power to motors
        launcher.setPower(0);
    }
}



