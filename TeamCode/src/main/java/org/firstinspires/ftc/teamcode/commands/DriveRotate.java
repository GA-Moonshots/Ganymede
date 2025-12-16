package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;

import org.firstinspires.ftc.teamcode.Ganymede;
import org.firstinspires.ftc.teamcode.utils.Constants;

/**
 * ╔═══════════════════════════════════════════════════════════════════════════╗
 * ║                        ROTATE COMMAND                                     ║
 * ║                                                                           ║
 * ║  Rotates the robot by a specified angle while maintaining position.        ║
 * ║                                                                           ║
 * ║  IMPORTANT: Pedro Pathing uses RADIANS, not degrees!                      ║
 * ║  This command accepts degrees for convenience and converts internally.    ║
 * ╚═══════════════════════════════════════════════════════════════════════════╝
 */
public class DriveRotate extends DriveAbstract {

    // ============================================================
    //                     COMMAND STATE
    // ============================================================

    private final double rotationDegrees;
    private Pose targetPose;
    private boolean finished = false;

    // ============================================================
    //                     CONSTRUCTOR

    /**
     * Creates a command to rotate the robot by a specified angle.
     *
     * @param robot Main robot object containing all subsystems
     * @param rotationDegrees Angle to rotate in degrees
     *                        Positive = counter-clockwise (left)
     *                        Negative = clockwise (right)
     * @param timeoutSeconds Safety timeout in seconds
     */
    public DriveRotate(Ganymede robot, double rotationDegrees, double timeoutSeconds) {
        super(robot, timeoutSeconds);
        this.rotationDegrees = rotationDegrees;
    }

    // ============================================================
    //                     COMMAND LIFECYCLE

    @Override
    public void initialize() {
        finished = false;
        timer.start();

        Pose currentPose = drive.getPose();

        // Calculate target heading - use currentPose.getHeading() directly
        double currentHeadingRad = currentPose.getHeading();
        double rotationRad = Math.toRadians(rotationDegrees);
        double targetHeadingRad = drive.toPedroHeading(currentHeadingRad + rotationRad);

        // Create target pose with same X/Y, new heading
        targetPose = new Pose(
                currentPose.getX(),
                currentPose.getY(),
                targetHeadingRad
        );

        // Build and follow the path
        PathBuilder builder = new PathBuilder(follower)
                .addPath(new BezierCurve(currentPose, targetPose));
        follower.followPath(builder.build());

        // Telemetry for debugging
        robot.telemetry.addData("DriveRotate", "Initialized");
        robot.telemetry.addData("Current Heading", "%.1f°", Math.toDegrees(currentHeadingRad));
        robot.telemetry.addData("Rotation", "%.1f°", rotationDegrees);
        robot.telemetry.addData("Target Heading", "%.1f°", Math.toDegrees(targetHeadingRad));
    }

    @Override
    public void execute() {
        // Check if we've reached target heading within tolerance
        // Use a loose heading tolerance since Pedro's heading control can be finicky
        if (follower.atPose(targetPose, Constants.POSE_TOLERANCE, Constants.POSE_TOLERANCE)) {
            finished = true;
        }

        // Update telemetry
        Pose current = drive.getPose();
        double headingError = Math.abs(targetPose.getHeading() - current.getHeading());

        robot.telemetry.addData("Current Heading", "%.1f°", Math.toDegrees(current.getHeading()));
        robot.telemetry.addData("Target Heading", "%.1f°", Math.toDegrees(targetPose.getHeading()));
        robot.telemetry.addData("Heading Error", "%.1f°", Math.toDegrees(headingError));
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        standardCleanup();
        drive.follower.breakFollowing();

        if (interrupted) {
            robot.telemetry.addData("DriveRotate", "INTERRUPTED");
        } else {
            robot.telemetry.addData("DriveRotate", "Complete!");
        }
    }

    @Override
    public boolean isFinished() {
        return finished || timer.done();
    }
}