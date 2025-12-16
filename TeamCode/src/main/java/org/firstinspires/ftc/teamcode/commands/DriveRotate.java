package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;

import org.firstinspires.ftc.teamcode.Ganymede;

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
    private static final double ROTATION_TOLERANCE_DEGREES = 10.0;

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

        // Get current heading in radians
        double currentHeadingRad = currentPose.getHeading();

        // Convert rotation from degrees to radians, then add to current heading
        double rotationRad = Math.toRadians(rotationDegrees);
        double targetHeadingRad = drive.toPedroHeading(currentHeadingRad + rotationRad);

        // Create target pose with same position but new heading
        targetPose = new Pose(
                currentPose.getX(),
                currentPose.getY(),
                targetHeadingRad
        );

        // Build and follow the path
        PathBuilder builder = new PathBuilder(follower)
                .addPath(new BezierCurve(currentPose, targetPose));
        follower.followPath(builder.build());

        // Debug telemetry
        robot.telemetry.addData("DriveRotate", "Initialized");
        robot.telemetry.addData("Current Heading", "%.1f°", Math.toDegrees(currentHeadingRad));
        robot.telemetry.addData("Rotation", "%.1f°", rotationDegrees);
        robot.telemetry.addData("Target Heading", "%.1f°", Math.toDegrees(targetHeadingRad));
    }

    @Override
    public void execute() {
        // Get current heading directly from pose
        Pose currentPose = drive.getPose();
        double currentHeading = currentPose.getHeading();
        double targetHeading = targetPose.getHeading();

        // CRITICAL: Normalize both to [-π, π] before comparing
        currentHeading = normalizeAngle(currentHeading);
        targetHeading = normalizeAngle(targetHeading);

        // Calculate heading error (shortest angular distance)
        double headingError = angleDifference(targetHeading, currentHeading);
        double headingErrorDegrees = Math.toDegrees(Math.abs(headingError));

        // Check if we're within tolerance
        if (headingErrorDegrees < ROTATION_TOLERANCE_DEGREES) {
            finished = true;
            // Don't break following here - let Pedro finish naturally
        }

        // Real-time telemetry for debugging
        robot.sensors.addTelemetry("Current Heading", "%.1f°", Math.toDegrees(currentHeading));
        robot.sensors.addTelemetry("Target Heading", "%.1f°", Math.toDegrees(targetHeading));
        robot.sensors.addTelemetry("Heading Error", "%.1f°", headingErrorDegrees);
        robot.sensors.addTelemetry("Is Busy", String.valueOf(follower.isBusy()));
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);

        // Break following and stop motors
        follower.breakFollowing();
        drive.stop();

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
     * Normalizes angle to [-π, π] range.
     *
     * @param angle Angle in radians
     * @return Normalized angle in range [-π, π]
     */
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        while (angle <= -Math.PI) {
            angle += 2 * Math.PI;
        }
        return angle;
    }

    /**
     * Calculates the shortest angular difference between two angles.
     * Properly handles wrap-around at ±180°.
     *
     * @param targetAngle Target angle in radians (should be normalized)
     * @param currentAngle Current angle in radians (should be normalized)
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