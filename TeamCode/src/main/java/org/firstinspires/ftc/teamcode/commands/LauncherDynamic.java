package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Ganymede;

public class LauncherDynamic extends DriveAbstract {
    private boolean team;

    // Defined Poses
    // THESE ARE PLACEHOLDER VALUES
    private final Pose BLUE_TARGET_POSE = new Pose(60, 88, 90);
    private final Pose RED_TARGET_POSE = new Pose(60, 88, 90);

    // 1 -> Blue, 0 -> Purple
    public LauncherDynamic(Ganymede robot, boolean team, double timeoutSeconds) {
        super(robot, timeoutSeconds);
        this.team = team;
    }

    @Override
    public void execute() {

    }

    public double calculateShooterPower() {
        double delta_x, delta_y;
        double g = 9.81;
        double y = 12; // dummy value
        double theta = 90; // dummy value
        double k = 5; // dummy value

        // real time calculation
        // check for -x and -y
        if (team) {
            delta_x = this.drive.getPose().getX() - BLUE_TARGET_POSE.getX();
            delta_y = this.drive.getPose().getY() - BLUE_TARGET_POSE.getY();
        } else {
            delta_x = this.drive.getPose().getX() - RED_TARGET_POSE.getX();
            delta_y = this.drive.getPose().getY() - RED_TARGET_POSE.getY();
        }

        double x = Math.sqrt(Math.pow(delta_x, 2) + Math.pow(delta_y, 2));

        double inside = -g/((y-x*Math.tan(theta)) * 2 * Math.pow(Math.cos(theta), 2));
        return (1/k) * Math.sqrt(inside);

    }
}
