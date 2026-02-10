package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.Ganymede;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.Constants;

import java.util.concurrent.TimeUnit;

/**
 * ╔═══════════════════════════════════════════════════════════════════════════╗
 * ║                      DYNAMIC LAUNCHER COMMAND                             ║
 * ║                                                                           ║
 * ║  Calculates optimal launcher power based on robot's current position      ║
 * ║  and distance to goal, then fires with the computed power level.           ║
 * ║                                                                           ║
 * ║  This command:                                                            ║
 * ║    1. Calculates distance from robot to alliance goal                     ║
 * ║    2. Uses projectile motion kinematics to determine required velocity    ║
 * ║    3. Converts velocity to motor power (0.0 - 1.0)                        ║
 * ║    4. Fires the launcher at the calculated power via LauncherRPM          ║
 * ║                                                                           ║
 * ║  Usage: Can MUST combined with DriveTurnToGoal for full auto-targeting:     ║
 * ║    new SequentialCommandGroup(                                            ║
 * ║        new DriveTurnToGoal(robot, isGreen, 3),                            ║
 * ║        new LauncherDynamic(robot, isGreen, 10)                            ║
 * ║    )                                                                      ║
 * ╚═══════════════════════════════════════════════════════════════════════════╝
 */
public class LauncherDynamic extends DriveAbstract {

    // ============================================================
    //                     PHYSICAL CONSTANTS
    // ============================================================

    /**
     * Height of goal opening above field surface (in inches).
     * From Competition Manual: "The top lip of the GOAL is 38.75 in. (98.45 cm)
     * from the surface of the TILE."
     *
     * We subtract our launcher height to get the vertical delta.
     */
    private static final double GOAL_HEIGHT_INCHES = 38.75;

    /**
     * Height of our launcher exit point above the field (in inches).
     * TODO: Measure this on your actual robot!
     */
    private static final double LAUNCHER_HEIGHT_INCHES = 12.0;

    /**
     * Launcher angle from horizontal (in degrees).
     * TODO: Measure your launcher's fixed angle!
     */
    private static final double LAUNCHER_ANGLE_DEGREES = 45.0;

    /**
     * Velocity-to-power conversion factor.
     * This converts the calculated m/s velocity to motor power (0-1).
     *
     * Determined experimentally: fire at known powers, measure ball velocity,
     * then calculate k = velocity / power.
     *
     * TODO: Tune this through testing! Start with estimate, then refine.
     */
    private static final double VELOCITY_TO_POWER_FACTOR = 8.0;

    /**
     * Minimum and maximum allowed power values.
     * Prevents the calculation from outputting unrealistic values.
     */
    private static final double MIN_POWER = 0.5;
    private static final double MAX_POWER = 1.0;

    // ============================================================
    //                     COMMAND STATE
    // ============================================================

    /** Are we launching a green ball? (affects turret position check) */
    private final boolean isGreen;

    /** Calculated power level for the launcher */
    private double calculatedPower;

    /** Distance to goal (for telemetry) */
    private double distanceToGoal;

    /** Has the launch sequence been triggered? */
    private boolean launchTriggered = false;

    /** Timer for the overall command */
    private Timing.Timer launchTimer;

    // ============================================================
    //                     CONSTRUCTOR
    // ============================================================

    /**
     * Creates a dynamic launcher command that calculates and applies optimal power.
     *
     * @param robot Main robot object
     * @param isGreen true if launching green ball (turret LEFT), false for purple (turret FRONT)
     * @param timeoutSeconds Safety timeout - should allow time for power calc + launch
     */
    public LauncherDynamic(Ganymede robot, boolean isGreen, double timeoutSeconds) {
        super(robot, timeoutSeconds);
        this.isGreen = isGreen;
    }

    // ============================================================
    //                     COMMAND LIFECYCLE
    // ============================================================

    @Override
    public void initialize() {
        super.initialize();
        timer.start();
        launchTriggered = false;

        // Calculate distance and power immediately on init
        distanceToGoal = calculateDistanceToGoal();
        calculatedPower = calculateShooterPower();

        // Clamp to safe range
        calculatedPower = Math.max(MIN_POWER, Math.min(MAX_POWER, calculatedPower));

        // Telemetry for debugging
        robot.sensors.addTelemetry("═══ Dynamic Launcher ═══", "");
        robot.sensors.addTelemetry("Ball Color", isGreen ? "GREEN" : "PURPLE");
        robot.sensors.addTelemetry("Alliance", robot.isRed ? "RED" : "BLUE");
        robot.sensors.addTelemetry("Distance to Goal", "%.1f inches", distanceToGoal);
        robot.sensors.addTelemetry("Calculated Power", "%.3f", calculatedPower);
    }

    @Override
    public void execute() {
        // Trigger the launch once (LauncherRPM handles RPM-based feeding)
        if (!launchTriggered) {
            // Schedule LauncherRPM with our calculated power
            new LauncherRPM(robot, calculatedPower).schedule();
            launchTriggered = true;

            robot.sensors.addTelemetry("Launch Status", "FIRING at %.2f power", calculatedPower);
        }

        // Update telemetry while waiting
        robot.sensors.addTelemetry("Time Elapsed", "%.1f sec", timer.elapsedTime());
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        robot.sensors.addTelemetry("Dynamic Launch", interrupted ? "INTERRUPTED" : "COMPLETE");
    }

    @Override
    public boolean isFinished() {
        // Let the LauncherRPM command handle its own RPM-based logic
        // We just provide a safety timeout
        return timer.done();
    }

    // ============================================================
    //                     DISTANCE CALCULATION
    // ============================================================

    /**
     * Calculates the straight-line distance from robot to the target goal.
     * Uses the alliance color to determine which goal we're aiming at.
     *
     * @return Distance to goal in inches
     */
    public double calculateDistanceToGoal() {
        // Get current robot position
        Pose currentPose = drive.getPose();

        // Get target goal coordinates based on alliance
        double targetX = robot.isRed ? Constants.RED_TARGET_X : Constants.BLUE_TARGET_X;
        double targetY = robot.isRed ? Constants.RED_TARGET_Y : Constants.BLUE_TARGET_Y;

        // Calculate deltas
        double deltaX = targetX - currentPose.getX();
        double deltaY = targetY - currentPose.getY();

        // Pythagorean theorem for straight-line distance
        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }

    // ============================================================
    //                     KINEMATICS CALCULATION
    //                    (Student's Original Math)
    // ============================================================

    /**
     * Calculates the required launcher power based on projectile motion physics.
     *
     * ┌─────────────────────────────────────────────────────────────────────────┐
     * │  PROJECTILE MOTION PHYSICS EXPLANATION                                  │
     * │                                                                         │
     * │  We're solving for initial velocity (v₀) needed to hit a target at:     │
     * │    • Horizontal distance: x (calculated from robot to goal)             │
     * │    • Vertical height: y (goal height - launcher height)                 │
     * │    • Launch angle: θ (our fixed launcher angle)                          │
     * │                                                                         │
     * │  The standard projectile motion equations are:                          │
     * │    x = v₀ × cos(θ) × t                    (horizontal motion)           │
     * │    y = v₀ × sin(θ) × t - ½ × g × t²       (vertical motion)             │
     * │                                                                         │
     * │  Solving for t from the first equation:                                  │
     * │    t = x / (v₀ × cos(θ))                                                │
     * │                                                                         │
     * │  Substituting into the second equation and solving for v₀:              │
     * │    y = x × tan(θ) - (g × x²) / (2 × v₀² × cos²(θ))                      │
     * │                                                                         │
     * │  Rearranging for v₀²:                                                   │
     * │    v₀² = (g × x²) / (2 × cos²(θ) × (x × tan(θ) - y))                    │
     * │                                                                         │
     * │  Or equivalently (the form used below):                                 │
     * │    v₀² = g / ((x×tan(θ) - y) × 2 × cos²(θ))      [note the sign!]      │
     * │                                                                         │
     * │  The negative sign appears because (y - x×tan(θ)) is typically          │
     * │  negative when shooting upward at a target (the ball rises then falls). │
     * │                                                                         │
     * │  Finally, we convert velocity to motor power using an empirically       │
     * │  determined conversion factor k:                                        │
     * │    power = v₀ / k                                                       │
     * └─────────────────────────────────────────────────────────────────────────┘
     *
     * @return Motor power level (0.0 to 1.0) needed to reach the goal
     */
    public double calculateShooterPower() {
        // ────────────────────────────────────────────────────────────
        // PHYSICAL CONSTANTS
        // ────────────────────────────────────────────────────────────

        // Gravitational acceleration (m/s²)
        // Using metric internally for physics calculations
        double g = 9.81;

        // Vertical height difference: goal height minus launcher height
        // Convert inches to meters for physics calculation
        double y_inches = GOAL_HEIGHT_INCHES - LAUNCHER_HEIGHT_INCHES;
        double y = y_inches * 0.0254;  // inches to meters

        // Launch angle in radians (physics functions use radians)
        double theta = Math.toRadians(LAUNCHER_ANGLE_DEGREES);

        // Velocity-to-power conversion factor
        // Higher k = more velocity per unit power
        double k = VELOCITY_TO_POWER_FACTOR;

        // ────────────────────────────────────────────────────────────
        // DISTANCE CALCULATION
        // ────────────────────────────────────────────────────────────

        // Get current position and calculate distance to goal
        Pose currentPose = this.drive.getPose();

        // Get target based on alliance
        double targetX = robot.isRed ? Constants.RED_TARGET_X : Constants.BLUE_TARGET_X;
        double targetY = robot.isRed ? Constants.RED_TARGET_Y : Constants.BLUE_TARGET_Y;

        // Calculate position deltas
        double delta_x = currentPose.getX() - targetX;
        double delta_y = currentPose.getY() - targetY;

        // Horizontal distance to goal (Pythagorean theorem)
        // Convert from inches to meters for physics calculation
        double r_inches = Math.sqrt(delta_x * delta_x + delta_y * delta_y);
        double r = r_inches * 0.0254;  // inches to meters

        // ────────────────────────────────────────────────────────────
        // PROJECTILE MOTION FORMULA
        // ────────────────────────────────────────────────────────────
        //
        // This is the rearranged projectile motion equation solving for v₀²:
        //
        //   v₀² = -g / ((y - x×tan(θ)) × 2 × cos²(θ))
        //
        // The "inside" variable holds the expression under the square root:
        //   inside = -g / ((y - x×tan(θ)) × 2 × cos²(θ))
        //
        // Then v₀ = √(inside)
        // And power = v₀ / k

        double tanTheta = Math.tan(theta);
        double cosTheta = Math.cos(theta);
        double cosThetaSquared = cosTheta * cosTheta;

        // Denominator: (y - x×tan(θ)) × 2 × cos²(θ)
        double denominator = (r * tanTheta - y) * 2 * cosThetaSquared;

        // The full expression: -g / denominator
        // Note: denominator is typically negative (ball must rise then fall),
        // so -g / (negative) gives positive v₀²
        double inside = g / denominator;

        // ────────────────────────────────────────────────────────────
        // SAFETY CHECK
        // ────────────────────────────────────────────────────────────

        // If 'inside' is negative, the target is unreachable at this angle
        // (would require imaginary velocity). Return max power as fallback.
        if (inside < 0) {
            robot.sensors.addTelemetry("KINEMATICS WARNING",
                    "Target unreachable at θ=%.0f°, using max power", LAUNCHER_ANGLE_DEGREES);
            return MAX_POWER;
        }

        // ────────────────────────────────────────────────────────────
        // FINAL CALCULATION
        // ────────────────────────────────────────────────────────────

        // v₀ = √(inside)  [this is the required initial velocity in m/s]
        double velocity = Math.sqrt(inside);

        // Convert velocity to motor power using empirical factor
        // power = velocity / k
        double power = velocity / k;

        // Debug telemetry for tuning
        robot.sensors.addTelemetry("Kinematics Debug", "r=%.2fm, y=%.2fm, v₀=%.2fm/s",
                r, y, velocity);

        return power;
    }

    // ============================================================
    //                     PUBLIC GETTERS
    // ============================================================

    /**
     * Returns the calculated power level (for telemetry or chaining).
     * Only valid after initialize() has been called.
     */
    public double getCalculatedPower() {
        return calculatedPower;
    }

    /**
     * Returns the distance to goal (for telemetry or chaining).
     * Only valid after initialize() has been called.
     */
    public double getDistanceToGoal() {
        return distanceToGoal;
    }

}