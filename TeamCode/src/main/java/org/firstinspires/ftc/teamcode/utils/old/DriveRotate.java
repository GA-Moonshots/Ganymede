package org.firstinspires.ftc.teamcode.utils.old;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.Ganymede;
import org.firstinspires.ftc.teamcode.commands.DriveAbstract;

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
    private double targetHeadingRad;
    private boolean finished = false;
    private static final double ROTATION_TOLERANCE_DEGREES = 10.0;

    // ============================================================
    //                     CONSTRUCTOR

    /**
     * Creates a command to rotate the robot by a specified angle.
     *
     * @param robot Main robot object containing all subsystems
     * @param rotationDegrees Angle (in degrees) to add to current heading
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

        // Use the normalized heading (preferred over raw getPose().getHeading())
        double currentHeadingRad = drive.getNormalizedHeading();

        // Convert rotation from degrees to radians, then add to current heading
        double rotationRad = Math.toRadians(rotationDegrees);
        targetHeadingRad = normalizeAngle(currentHeadingRad + rotationRad);

        // Path-based in-place turn: followPath mode uses Pedro's full unscaled PID suite.
        // turnTo() uses holdPoint() which applies holdPointHeadingScaling (designed for
        // stationary settling, not active rotation) and fights translational drift mid-turn.
        Pose targetPose = new Pose(
                currentPose.getX() + 0.1 * Math.cos(currentHeadingRad),
                currentPose.getY() + 0.1 * Math.sin(currentHeadingRad),
                targetHeadingRad
        );
        Path turnPath = new Path(new BezierLine(currentPose, targetPose));
        turnPath.setLinearHeadingInterpolation(currentHeadingRad, targetHeadingRad, 0.8);
        follower.followPath(turnPath);

        // Debug telemetry
        robot.telemetry.addData("DriveRotate", "Initialized");
        robot.telemetry.addData("Current Heading", "%.1f°", Math.toDegrees(currentHeadingRad));
        robot.telemetry.addData("Rotation", "%.1f°", rotationDegrees);
        robot.telemetry.addData("Target Heading", "%.1f°", Math.toDegrees(targetHeadingRad));
        robot.telemetry.update();
    }

    @Override
    public void execute() {
        double currentHeading = drive.getNormalizedHeading();
        double targetHeading = normalizeAngle(targetHeadingRad);

        // Calculate heading error (shortest angular distance)
        double headingError = angleDifference(targetHeading, currentHeading);
        double headingErrorDegrees = Math.toDegrees(Math.abs(headingError));

        // Check if we're within tolerance
        if (headingErrorDegrees < ROTATION_TOLERANCE_DEGREES) {
            finished = true;
        }

        // Real-time telemetry for debugging
        robot.sensors.addTelemetry("Current Heading", "%.1f°", Math.toDegrees(currentHeading));
        robot.sensors.addTelemetry("Target Heading", "%.1f°", Math.toDegrees(targetHeading));
        robot.sensors.addTelemetry("Heading Error", "%.1f°", headingErrorDegrees);
        robot.sensors.addTelemetry("Is Busy", String.valueOf(follower.isBusy()));
        robot.sensors.addTelemetry("Is Turning", String.valueOf(follower.isTurning()));
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