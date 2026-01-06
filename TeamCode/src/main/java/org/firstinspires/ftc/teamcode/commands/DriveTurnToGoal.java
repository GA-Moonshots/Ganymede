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
 * ║    • Ball color being launched (green vs purple)                          ║
 * ║    • Turret position (FRONT for purple, LEFT for green)                   ║
 * ║                                                                           ║
 * ║  GREEN vs PURPLE targeting:                                               ║
 * ║    • PURPLE: Turret in FRONT position → aim directly at goal              ║
 * ║    • GREEN:  Turret in LEFT position → aim with 90° offset                ║
 * ║                                                                           ║
 * ║  Rotation compensation:                                                   ║
 * ║    Our robot's rotation isn't perfect, so we over-compensate by a         ║
 * ║    tunable factor. Different alliances may need different values.         ║
 * ║                                                                           ║
 * ║  Uses PedroDrive's angle translators for proper ±180° wrap-around.        ║
 * ╚═══════════════════════════════════════════════════════════════════════════╝
 */
public class DriveTurnToGoal extends DriveAbstract {

    // ============================================================
    //                     TUNING CONSTANTS
    // ============================================================

    /**
     * Over-compensation factor for rotation accuracy.
     * Our robot tends to under-rotate, so we overshoot by this percentage.
     * 1.0 = no compensation, 1.1 = 10% overshoot, etc.
     *
     * TODO: Tune these values on the actual field!
     */
    private static final double RED_ROTATION_COMPENSATION = 1.08;   // Red alliance tends to need ~8% more
    private static final double BLUE_ROTATION_COMPENSATION = 1.12;  // Blue alliance needs ~12% more

    /**
     * Additional heading offset for GREEN ball launches (in degrees).
     * When turret is LEFT, the launcher points 90° from robot heading,
     * so we need to rotate the robot to compensate.
     */
    private static final double GREEN_HEADING_OFFSET_DEGREES = 90.0;

    // ============================================================
    //                     COMMAND STATE
    // ============================================================

    /** Target heading to face the goal (in radians) */
    private double targetHeading;

    /** Current robot pose */
    private Pose currentPose;

    /** Flag to track when turn is complete */
    private boolean finished = false;

    /** Are we targeting for a green ball? (false = purple) */
    private final boolean isGreen;

    // ============================================================
    //                     CONSTRUCTORS
    // ============================================================

    /**
     * Creates a command to turn the robot to face the goal for PURPLE ball launch.
     * This is the default constructor - purple balls use FRONT turret position.
     *
     * @param robot Main robot object
     * @param timeoutSeconds Safety timeout in seconds (typically 3-5s)
     */
    public DriveTurnToGoal(Ganymede robot, double timeoutSeconds) {
        this(robot, false, timeoutSeconds);  // Default to purple
    }

    /**
     * Creates a command to turn the robot to face the goal for a specific ball color.
     *
     * The command automatically:
     * 1. Determines target goal based on robot.isRed
     * 2. Calculates heading from current position to goal
     * 3. Applies GREEN offset if launching green ball (turret LEFT)
     * 4. Applies rotation compensation for our imprecise turning
     * 5. Turns robot to face the calculated target
     *
     * @param robot Main robot object
     * @param isGreen true for green ball (turret LEFT), false for purple (turret FRONT)
     * @param timeoutSeconds Safety timeout in seconds (typically 3-5s)
     */
    public DriveTurnToGoal(Ganymede robot, boolean isGreen, double timeoutSeconds) {
        super(robot, timeoutSeconds);
        this.isGreen = isGreen;
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

        // Apply heading offset based on ball color (green needs 90° offset)
        // Then apply rotation compensation for our imprecise turning
        targetHeading = calculateTargetHeading(headingToGoal);

        // ============================================================
        // PEDRO PATHING IN-PLACE TURN TECHNIQUE
        // Pedro doesn't have a direct "turn to heading" method, so we create
        // a very short path (0.1 inches) with linear heading interpolation.
        // This causes the robot to turn while barely moving forward.

        double currentHeading = drive.getNormalizedHeading();

        // Create target pose: Same position + tiny forward offset + new heading
        Pose targetPose = new Pose(
                currentPose.getX() + 0.1 * Math.cos(currentHeading),
                currentPose.getY() + 0.1 * Math.sin(currentHeading),
                targetHeading  // Already normalized by calculateTargetHeading()
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
        robot.sensors.addTelemetry("Ball Color", isGreen ? "GREEN" : "PURPLE");
        robot.sensors.addTelemetry("Turret Expected", isGreen ? "LEFT" : "FRONT");
        robot.sensors.addTelemetry("Current Position", "X:%.1f Y:%.1f",
                currentPose.getX(), currentPose.getY());
        robot.sensors.addTelemetry("Target Goal", "X:%.1f Y:%.1f", targetX, targetY);
        robot.sensors.addTelemetry("Raw Heading to Goal", "%.1f°", Math.toDegrees(headingToGoal));
        robot.sensors.addTelemetry("Compensated Target", "%.1f°", Math.toDegrees(targetHeading));
    }

    @Override
    public void execute() {
        super.execute();

        // Get current heading using PedroDrive's normalized method
        double currentHeading = drive.getNormalizedHeading();

        // Calculate heading error (shortest angular distance)
        double headingError = angleDifference(targetHeading, currentHeading);
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
     * Calculates the target heading with all adjustments applied.
     *
     * Processing order:
     * 1. Start with raw heading to goal
     * 2. Apply GREEN offset if launching green ball (turret is LEFT, so robot
     *    needs to face 90° away from goal for launcher to point at goal)
     * 3. Apply rotation compensation (we tend to under-rotate)
     * 4. Normalize to Pedro's expected range
     *
     * @param headingToGoal The direct heading from robot to goal (in radians)
     * @return The fully adjusted target heading (in radians)
     */
    private double calculateTargetHeading(double headingToGoal) {
        double adjustedHeading = headingToGoal;

        // Step 1: Apply GREEN offset if needed
        // When turret is LEFT, launcher points 90° left of robot heading
        // So robot needs to face 90° RIGHT of goal for launcher to aim at goal
        if (isGreen) {
            double offsetRadians = Math.toRadians(GREEN_HEADING_OFFSET_DEGREES);
            adjustedHeading = headingToGoal - offsetRadians;  // Subtract to rotate right
        }

        // Step 2: Apply rotation compensation
        // Calculate the rotation delta from current heading
        double currentHeading = drive.getNormalizedHeading();
        double rotationDelta = angleDifference(adjustedHeading, currentHeading);

        // Apply alliance-specific compensation factor
        double compensation = robot.isRed ? RED_ROTATION_COMPENSATION : BLUE_ROTATION_COMPENSATION;
        double compensatedDelta = rotationDelta * compensation;

        // Apply compensated rotation to current heading
        adjustedHeading = currentHeading + compensatedDelta;

        // Step 3: Normalize through PedroDrive's output translator
        return drive.toPedroHeading(adjustedHeading);
    }

    /**
     * Calculates the shortest angular difference between two angles.
     *
     * This properly handles the wrap-around at ±180°. For example:
     * - angleDifference(170°, -170°) = 20° (not 340°)
     * - angleDifference(-170°, 170°) = -20° (not -340°)
     *
     * The result is positive for counter-clockwise rotation,
     * negative for clockwise rotation.
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

    // ============================================================
    //                     PUBLIC GETTERS
    // ============================================================

    /**
     * Returns whether this command is targeting for a green ball.
     * Useful for sequencing with LauncherDynamic.
     */
    public boolean isTargetingGreen() {
        return isGreen;
    }
}