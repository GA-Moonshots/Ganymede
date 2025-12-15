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

        // Check if we've reached target within tolerance
        // Position tolerance in inches, heading tolerance in radians
        double headingToleranceRad = Math.toRadians(Constants.HEADING_TOLERANCE_DEGREES);
        if (follower.atPose(targetPose, Constants.POSE_TOLERANCE, headingToleranceRad)) {
            finished = true;
        }

        // Debug telemetry
        Pose currentPose = drive.getPose();
        double headingError = Math.toDegrees(
                Math.abs(targetPose.getHeading() - currentPose.getHeading())
        );
        robot.telemetry.addData("Heading Error", "%.1f°", headingError);
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
}