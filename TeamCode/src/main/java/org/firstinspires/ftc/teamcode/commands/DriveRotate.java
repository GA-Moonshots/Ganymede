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
        finished = false;  // ← CRITICAL: Reset state
        timer.start();

        // Get FRESH pose data from Pedro's follower
        Pose currentPose = drive.getPose();

        // Calculate target heading in radians
        // Use currentPose.getHeading() directly - it's already Pedro's heading
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

        // Persistent telemetry (won't scroll away)
        robot.sensors.addTelemetry("DriveRotate", "ACTIVE");
        robot.sensors.addTelemetry("Rotation", String.format("%.1f°", rotationDegrees));
    }

    @Override
    public void execute() {
        super.execute();  // ← CRITICAL: Call parent's execute

        // Check if follower thinks we're at the target pose
        // Use HEADING_TOLERANCE_DEGREES for the heading check
        double headingToleranceInches = Constants.HEADING_TOLERANCE_DEGREES * 0.1; // Convert to "distance"

        if (follower.atPose(targetPose, Constants.POSE_TOLERANCE, headingToleranceInches)) {
            finished = true;
        }

        // Update telemetry - only current/target heading (persistent display)
        Pose current = drive.getPose();
        robot.sensors.addTelemetry("Current Hdg", String.format("%.1f°", Math.toDegrees(current.getHeading())));
        robot.sensors.addTelemetry("Target Hdg", String.format("%.1f°", Math.toDegrees(targetPose.getHeading())));
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);  // ← Call parent first
        standardCleanup();       // ← Then cleanup

        // Break following to release control
        drive.follower.breakFollowing();

        finished = false;  // ← Reset for next time

        // Final telemetry
        robot.sensors.addTelemetry("DriveRotate", interrupted ? "INTERRUPTED" : "COMPLETE");
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