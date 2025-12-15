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
        timer.start();

        Pose currentPose = drive.getPose();

        // Get current heading in radians
        double currentHeadingRad = drive.getNormalizedHeading();

        // Convert rotation from degrees to radians, then add to current heading
        double rotationRad = Math.toRadians(rotationDegrees);
        double targetHeadingRad = drive.toPedroHeading(currentHeadingRad + rotationRad);

        // Create target pose with same position but new heading
        targetPose = new Pose(
                currentPose.getX(),
                currentPose.getY(),
                targetHeadingRad  // Already in radians, already normalized
        );

        // Build and follow the path
        PathBuilder builder = new PathBuilder(follower)
                .addPath(new BezierCurve(currentPose, targetPose));
        follower.followPath(builder.build());

        // ← REMOVE THIS LINE! Don't set finished = true here!
        // finished = true;

        // Debug telemetry
        robot.telemetry.addData("DriveRotate", "Initialized");
        robot.telemetry.addData("Current Heading", "%.1f°", Math.toDegrees(currentHeadingRad));
        robot.telemetry.addData("Rotation", "%.1f°", rotationDegrees);
        robot.telemetry.addData("Target Heading", "%.1f°", Math.toDegrees(targetHeadingRad));
    }

    @Override
    public void execute() {
        // Note: follower.update() is called by PedroDrive.periodic()
        // We don't call it here to avoid double-updating

        // Get current heading using PedroDrive's normalized method
        double currentHeading = drive.getNormalizedHeading();

        // Calculate heading error (shortest angular distance)
        double headingError = angleDifference(targetPose.getHeading(), currentHeading);
        double headingErrorDegrees = Math.toDegrees(Math.abs(headingError));

        // Check if we're within 2 degrees tolerance
        if (headingErrorDegrees < 2.0) {
            finished = true;
        }

        // Real-time telemetry for debugging
        robot.sensors.addTelemetry("Current Heading", "%.1f°", Math.toDegrees(currentHeading));
        robot.sensors.addTelemetry("Target Heading", "%.1f°", Math.toDegrees(targetPose.getHeading()));
        robot.sensors.addTelemetry("Heading Error", "%.1f°", headingErrorDegrees);
    }

    @Override
    public void end(boolean interrupted) {
        standardCleanup();

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

    // ============================================================
    //                     HELPER METHODS
    // ============================================================

    /**
     * Calculates the shortest angular difference between two angles.
     * Properly handles wrap-around at ±180°.
     *
     * @param targetAngle Target angle in radians
     * @param currentAngle Current angle in radians
     * @return Shortest difference in radians, range [-π, π]
     */
    private double angleDifference(double targetAngle, double currentAngle) {
        double difference = targetAngle - currentAngle;

        // Normalize the difference to [-π, π]
        while (difference > Math.PI) {
            difference -= 2 * Math.PI;
        }
        while (difference <= -Math.PI) {
            difference += 2 * Math.PI;
        }

        return difference;
    }
}