package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.Ganymede;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.Constants;

/**
 * ╔═══════════════════════════════════════════════════════════════════════════╗
 * ║                      DRIVE TURN TO GOAL COMMAND                           ║
 * ║                                                                           ║
 * ║  Automatically turns the robot to face the correct goal based on:         ║
 * ║    • Alliance color (red vs blue)                                         ║
 * ║    • Launcher/turret position (front vs left)                             ║
 * ║                                                                           ║
 * ║  Heading calculation accounts for turret position:                        ║
 * ║    • FRONT position: Aim directly at goal                                 ║
 * ║    • LEFT position:  Aim with 90° offset (launcher is rotated)            ║
 * ╚═══════════════════════════════════════════════════════════════════════════╝
 */
public class DriveTurnToGoal extends DriveAbstract {

    // ============================================================
    //                     COMMAND STATE
    // ============================================================

    /** Target heading to face the goal (in radians) */
    private double targetHeading;

    /** Current robot pose */
    private Pose currentPose;

    /** Flag to track when turn is complete */
    private boolean finished = false;

    // ============================================================
    //                     CONSTRUCTOR
    // ============================================================

    /**
     * Creates a command to turn the robot to face the appropriate goal.
     *
     * The command automatically:
     * 1. Determines target goal based on robot.isRed
     * 2. Calculates heading from current position to goal
     * 3. Applies launcher offset if turret is in LEFT position
     * 4. Turns robot to face the target
     *
     * @param robot Main robot object
     * @param timeoutSeconds Safety timeout in seconds (typically 3-5s)
     */
    public DriveTurnToGoal(Ganymede robot, double timeoutSeconds) {
        super(robot, timeoutSeconds);
    }

    // ============================================================
    //                     COMMAND LIFECYCLE
    // ============================================================

    @Override
    public void initialize() {
        timer.start();

        // Get current robot position
        currentPose = follower.getPose();

        // Determine which goal to target based on alliance color
        double targetX = robot.isRed ? Constants.RED_TARGET_X : Constants.BLUE_TARGET_X;
        double targetY = robot.isRed ? Constants.RED_TARGET_Y : Constants.BLUE_TARGET_Y;

        // Calculate heading to goal from current position
        double deltaX = targetX - currentPose.getX();
        double deltaY = targetY - currentPose.getY();
        double headingToGoal = Math.atan2(deltaY, deltaX);

        // Apply heading offset based on launcher/turret position
        targetHeading = calculateTargetHeading(headingToGoal);

        // ============================================================
        // PEDRO PATHING IN-PLACE TURN TECHNIQUE
        // ============================================================
        // Pedro doesn't have a direct "turn to heading" method, so we create
        // a very short path (0.1 inches) with linear heading interpolation.
        // This causes the robot to turn while barely moving forward.

        double currentHeading = currentPose.getHeading();

        // Create target pose: Same position + tiny forward offset + new heading
        Pose targetPose = new Pose(
                currentPose.getX() + 0.1 * Math.cos(currentHeading),
                currentPose.getY() + 0.1 * Math.sin(currentHeading),
                targetHeading
        );

        // Build path with linear heading interpolation
        // The 0.8 parameter means complete 80% of turn by path end (good for accuracy)
        Path turnPath = new Path(new BezierLine(currentPose, targetPose));
        turnPath.setLinearHeadingInterpolation(currentHeading, targetHeading, 0.8);

        // Follow the path at reduced speed for accuracy
        follower.setMaxPower(0.6);
        follower.followPath(turnPath);

        // Debug telemetry
        robot.sensors.addTelemetry("═══ Turn To Goal ═══", "");
        robot.sensors.addTelemetry("Alliance", robot.isRed ? "RED" : "BLUE");
        robot.sensors.addTelemetry("Turret Position", robot.turret.state.toString());
        robot.sensors.addTelemetry("Current Position", "X:%.1f Y:%.1f",
                currentPose.getX(), currentPose.getY());
        robot.sensors.addTelemetry("Target Goal", "X:%.1f Y:%.1f", targetX, targetY);
        robot.sensors.addTelemetry("Heading to Goal", "%.1f°", Math.toDegrees(headingToGoal));
        robot.sensors.addTelemetry("Target Heading", "%.1f°", Math.toDegrees(targetHeading));
    }

    @Override
    public void execute() {
        super.execute();

        // Get current heading
        double currentHeading = follower.getPose().getHeading();

        // Calculate heading error
        double headingError = normalizeAngle(targetHeading - currentHeading);
        double headingErrorDegrees = Math.toDegrees(Math.abs(headingError));

        // Check if we're within tolerance
        if (headingErrorDegrees < Constants.HEADING_TOLERANCE_DEGREES) {
            finished = true;
        }

        // Real-time telemetry for tuning
        robot.sensors.addTelemetry("Current Heading", "%.1f°", Math.toDegrees(currentHeading));
        robot.sensors.addTelemetry("Heading Error", "%.1f°", headingErrorDegrees);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);

        // Reset max power to default
        follower.setMaxPower(1.0);

        // Stop following path
        standardCleanup();

        // Final telemetry
        robot.sensors.addTelemetry("Turn Complete", interrupted ? "INTERRUPTED" : "FINISHED");
    }

    @Override
    public boolean isFinished() {
        return finished || timer.done();
    }

    // ============================================================
    //                     HELPER METHODS
    // ============================================================

    /**
     * Calculates the target heading based on the heading to goal and launcher position.
     *
     * Logic:
     * - If turret is in FRONT position: Aim directly at goal
     * - If turret is in LEFT position: Add 90° offset (launcher is rotated)
     *
     * @param headingToGoal The direct heading from robot to goal (in radians)
     * @return The adjusted target heading accounting for launcher position (in radians)
     */
    private double calculateTargetHeading(double headingToGoal) {
        // Check if turret is in LEFT position (launcher rotated 90°)
        if (robot.turret.state == Turret.TurretState.LEFT) {
            // Launcher is rotated 90° left, so robot needs to face 90° right of goal
            double offsetRadians = Math.toRadians(Constants.LAUNCHER_LEFT_HEADING_OFFSET_DEGREES);
            return normalizeAngle(headingToGoal - offsetRadians);
        } else {
            // FRONT position or MOVING: Aim directly at goal
            return headingToGoal;
        }
    }

    /**
     * Normalizes an angle to the range [-π, π].
     *
     * This ensures angle differences are calculated correctly,
     * handling the wraparound at ±180°.
     *
     * @param angle Angle in radians
     * @return Normalized angle in range [-π, π]
     */
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        while (angle < -Math.PI) {
            angle += 2 * Math.PI;
        }
        return angle;
    }
}