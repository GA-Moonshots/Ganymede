package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Ganymede;
import org.firstinspires.ftc.teamcode.utils.Constants;

/**
 * ╔═══════════════════════════════════════════════════════════════════════════╗
 * ║                        PEDRO DRIVE SUBSYSTEM                              ║
 * ║                                                                           ║
 * ║  Mecanum drivetrain with Pedro Pathing integration                        ║
 * ║                                                                           ║
 * ║  Features:                                                                ║
 * ║    • Field-centric and robot-centric drive modes                          ║
 * ║    • Integrated Pedro Pathing localization                                ║
 * ║    • Variable speed control for precision movements                       ║
 * ║    • IMU-based heading correction                                         ║
 * ║    • Live Panels dashboard visualization (TeleOp + Tuning)                ║
 * ║    • All motor configuration centralized in Constants.java                ║
 * ╚═══════════════════════════════════════════════════════════════════════════╝
 */
public class PedroDrive extends SubsystemBase {

    // ============================================================
    //                     CORE COMPONENTS
    // ============================================================

    /** Reference to main robot object for hardware access */
    private final Ganymede robot;
    /** Pedro Pathing Follower - handles all path following and localization */
    public final Follower follower;
    /** Individual motor controllers for mecanum drive */
    private final DcMotorEx leftFront, leftBack, rightFront, rightBack;
    /** IMU sensor for field-centric calculations and heading tracking */
    private final IMU imu;

    // ============================================================
    //                    DRIVE CONFIGURATION
    // ============================================================

    /** Toggle between field-centric (true) and robot-centric (false) drive modes */
    private boolean fieldCentric = Constants.DEFAULT_FIELD_CENTRIC;

    /** Speed multiplier for fine control (0.0 to 1.0) */
    private double driveSpeed = Constants.DEFAULT_DRIVE_SPEED;

    // ============================================================
    //                    PANELS DRAWING SETUP
    // ============================================================
    /** Robot radius for drawing on field (in inches) */
    private static final double ROBOT_RADIUS = 9.0;
    /** Panels field manager for drawing */
    private static FieldManager panelsField;
    private static final Style ROBOT_STYLE = new Style(
            "", "#4CAF50", 3.0  // Green - current position
    );
    /** Style for pose history trail (softer green) */
    private static final Style HISTORY_STYLE = new Style(
            "", "#81C784", 2.0  // Light green - history trail
    );

    /** Flag to enable/disable Panels drawing (can be toggled for performance) */
    private boolean panelsDrawingEnabled = true;

    /**
     * ┌─────────────────────────────────────────────────────────────────────┐
     * │                         CONSTRUCTOR                                 │
     * │                                                                     │
     * │  Initializes the Pedro Drive subsystem with motor configuration,     │
     * │  IMU setup, and Pedro Pathing follower initialization.              │
     * │  Also sets up Panels field drawing for live visualization.          │
     * │                                                                     │
     * │  All motor configuration is now pulled from Constants.java          │
     * └─────────────────────────────────────────────────────────────────────┘
     *
     * @param robot Reference to the main robot object
     * @param startPose Initial pose for localization (x, y in inches, heading in radians)
     */
    public PedroDrive(Ganymede robot, Pose startPose) {
        this.robot = robot;

        // ============ Motor Initialization ============
        leftFront = robot.hardwareMap.get(DcMotorEx.class, Constants.LEFT_FRONT_NAME);
        leftBack = robot.hardwareMap.get(DcMotorEx.class, Constants.LEFT_BACK_NAME);
        rightFront = robot.hardwareMap.get(DcMotorEx.class, Constants.RIGHT_FRONT_NAME);
        rightBack = robot.hardwareMap.get(DcMotorEx.class, Constants.RIGHT_BACK_NAME);

        // ============ Motor Configuration ============
        // All motor configuration now comes from Constants.java for easy machine-specific tuning

        // Set motor directions from Constants
        leftFront.setDirection(Constants.LEFT_FRONT_DIRECTION);
        leftBack.setDirection(Constants.LEFT_BACK_DIRECTION);
        rightFront.setDirection(Constants.RIGHT_FRONT_DIRECTION);
        rightBack.setDirection(Constants.RIGHT_BACK_DIRECTION);

        // Set zero power behavior from Constants
        leftFront.setZeroPowerBehavior(Constants.DRIVE_ZERO_POWER_BEHAVIOR);
        leftBack.setZeroPowerBehavior(Constants.DRIVE_ZERO_POWER_BEHAVIOR);
        rightFront.setZeroPowerBehavior(Constants.DRIVE_ZERO_POWER_BEHAVIOR);
        rightBack.setZeroPowerBehavior(Constants.DRIVE_ZERO_POWER_BEHAVIOR);

        // ============ IMU Initialization ============
        imu = robot.hardwareMap.get(IMU.class, Constants.IMU_NAME);
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        Constants.IMU_LOGO_DIRECTION,
                        Constants.IMU_USB_DIRECTION
                )
        );
        imu.initialize(parameters);

        // ============ Pedro Pathing Setup ============
        follower = Constants.createFollower(robot.hardwareMap);
        follower.setStartingPose(startPose);

        // ============ Panels Field Setup ============
        initializePanelsField();
    }

    /**
     * Initializes the Panels field for drawing robot position.
     * This is called once during construction and sets up the field offsets.
     */
    private void initializePanelsField() {
        try {
            if (panelsField == null) {
                panelsField = PanelsField.INSTANCE.getField();
                panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
            }
        } catch (Exception e) {
            // If Panels isn't available, disable drawing
            panelsDrawingEnabled = false;
            robot.sensors.addTelemetry("Panels Drawing", "Disabled (not available)");
        }
    }

    // ============================================================
    //                    PERIODIC UPDATES
    /**
     * Called automatically by the command scheduler every loop iteration.
     * Updates Pedro Pathing's localization system, posts telemetry,
     * and draws robot position to Panels dashboard.
     */

    @Override
    public void periodic() {
        // Update Pedro's localization system. DO NOT DUPLICATE THIS CALL
        update();

        // Draw robot position to Panels (TeleOp + Tuning)
        drawRobotToPanels();

        // Post basic data to driver station
        addTelemetry();
    }

    /**
     * Updates Pedro Pathing's internal localization calculations.
     * This method processes encoder readings and updates the robot's
     * estimated position on the field.
     */

    public void update() {
        follower.update();
    }

    // ============================================================
    //                    PANELS DRAWING
    /**
     * Draws the robot and its pose history to Panels dashboard.
     * This provides live field visualization during both TeleOp and tuning.
     * Won't conflict with tuning opmodes since they use their own Drawing class.
     */
    private void drawRobotToPanels() {
        if (!panelsDrawingEnabled || panelsField == null) {
            return;
        }
        try {
            Pose currentPose = follower.getPose();

            // Validate pose before drawing
            if (currentPose == null || Double.isNaN(currentPose.getX()) ||
                    Double.isNaN(currentPose.getY()) || Double.isNaN(currentPose.getHeading())) {
                return;
            }

            // Draw pose history trail
            drawPoseHistory(follower.getPoseHistory(), HISTORY_STYLE);

            // Draw current robot position
            drawRobot(currentPose, ROBOT_STYLE);

            // Send all drawing commands to Panels
            panelsField.update();
        } catch (Exception e) {
            // Silently catch any drawing errors to not interfere with robot operation
            panelsDrawingEnabled = false;
        }
    }

    /**
     * Draws the robot at a specified pose with heading indicator.
     * The heading is represented as a line showing robot orientation.
     *
     * @param pose Pose to draw the robot at
     * @param style Drawing style (color, stroke, etc.)
     */
    private void drawRobot(Pose pose, Style style) {
        if (pose == null || panelsField == null) return;

        // Draw robot body (circle)
        panelsField.setStyle(style);
        panelsField.moveCursor(pose.getX(), pose.getY());
        panelsField.circle(ROBOT_RADIUS);

        // Draw heading indicator (line showing front of robot)
        Vector headingVector = pose.getHeadingAsUnitVector();
        headingVector.setMagnitude(headingVector.getMagnitude() * ROBOT_RADIUS);

        double x1 = pose.getX() + headingVector.getXComponent() / 2;
        double y1 = pose.getY() + headingVector.getYComponent() / 2;
        double x2 = pose.getX() + headingVector.getXComponent();
        double y2 = pose.getY() + headingVector.getYComponent();

        panelsField.setStyle(style);
        panelsField.moveCursor(x1, y1);
        panelsField.line(x2, y2);
    }

    /**
     * Draws the robot's pose history as a trail on the field.
     * Shows where the robot has been during the match.
     *
     * @param poseHistory PoseHistory object containing position trail
     * @param style Drawing style for the trail
     */
    private void drawPoseHistory(PoseHistory poseHistory, Style style) {
        if (poseHistory == null || panelsField == null) return;

        panelsField.setStyle(style);

        double[] xPositions = poseHistory.getXPositionsArray();
        double[] yPositions = poseHistory.getYPositionsArray();

        if (xPositions == null || yPositions == null) return;

        int size = Math.min(xPositions.length, yPositions.length);

        // Draw lines connecting each position in the history
        for (int i = 0; i < size - 1; i++) {
            panelsField.moveCursor(xPositions[i], yPositions[i]);
            panelsField.line(xPositions[i + 1], yPositions[i + 1]);
        }
    }

    /**
     * Toggles Panels drawing on/off for performance tuning if needed.
     */
    public void togglePanelsDrawing() {
        panelsDrawingEnabled = !panelsDrawingEnabled;
        robot.sensors.addTelemetry("Panels Drawing", panelsDrawingEnabled ? "ENABLED" : "DISABLED");
    }

    /**
     * @return Whether Panels drawing is currently enabled
     */
    public boolean isPanelsDrawingEnabled() {
        return panelsDrawingEnabled;
    }

    // ============================================================
    //                    DRIVE CONTROL
    /**
     * Main teleop drive method using mecanum kinematics.
     * Supports both field-centric and robot-centric control modes.
     *
     * @param forward Forward/backward input (-1.0 to 1.0)
     * @param strafe  Left/right strafe input (-1.0 to 1.0)
     * @param turn    Rotation input (-1.0 to 1.0)
     */
    public void drive(double forward, double strafe, double turn) {
        // Note: Pedro uses opposite convention for robotCentric parameter
        follower.setTeleOpDrive(
                forward * driveSpeed,
                strafe * driveSpeed,
                turn * driveSpeed,
                !fieldCentric  // Inverted: fieldCentric=true means robotCentric=false
        );
    }


    /**
     * Emergency stop - immediately halts all drive motors.
     * Used for safety stops and at the end of autonomous paths.
     */
    public void stop() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    // ============================================================
    // ============================================================
    //                    LOCALIZATION

    /**
     * Returns the robot's current pose from Pedro's localization system.
     *
     * @return Current Pose (x, y in inches, heading in radians)
     */
    public Pose getPose() {
        return follower.getPose();
    }

    /**
     * Manually sets the robot's pose for relocalization.
     * Useful for vision-based corrections (e.g., AprilTag detection).
     *
     * @param newPose New pose to set (x, y in inches, heading in radians)
     */
    public void setPose(Pose newPose) {
        follower.setPose(newPose);
    }

    //============================================================//
    //                    UTILITY METHODS                         //

    /**
     * INPUT TRANSLATOR: Gets robot's heading normalized to [-π, π] radians.
     * Use this instead of getPose().getHeading() in commands that calculate
     * forward vectors or heading differences.
     *
     * @return Current robot heading in radians, normalized to [-π, π]
     */
    public double getNormalizedHeading() {
        double heading = follower.getPose().getHeading();
        return normalizeAngle(heading);
    }

    /**
     * OUTPUT TRANSLATOR: Converts our normalized heading to Pedro's format.
     * Use this when creating Pose objects for path targets.
     *
     * @param normalizedHeading Our heading in radians
     * @return Heading in Pedro's expected format ([-π, π])
     */
    public double toPedroHeading(double normalizedHeading) {
        return normalizeAngle(normalizedHeading);
    }

    /**
     * Core angle normalization - wraps any angle to [-π, π] radians.
     *
     * CRITICAL: Pedro Pathing uses RADIANS, not degrees!
     * - Math.PI = 180°
     * - -Math.PI = -180°
     *
     * @param angle Angle in radians (can be any value)
     * @return Normalized angle in range [-π, π]
     */
    private double normalizeAngle(double angle) {
        // Keep wrapping until angle is in range [-π, π]
        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        while (angle <= -Math.PI) {
            angle += 2 * Math.PI;
        }
        return angle;
    }

    /**
     * Resets the robot's heading to 0 degrees (facing forward).
     * Updates both IMU and Pedro Pathing's heading reference.
     */
    public void resetHeading() {
        imu.resetYaw();

        // Maintain position but reset heading in Pedro's localization
        Pose currentPose = getPose();
        setPose(new Pose(currentPose.getX(), currentPose.getY(), 0));
    }

    /**
     * Toggles between field-centric and robot-centric drive modes.
     * Updates telemetry to show current mode.
     */
    public void toggleFieldCentric() {
        fieldCentric = !fieldCentric;
    }

    /**
     * Sets the global drive speed multiplier for precision control.
     *
     * @param speed Desired speed multiplier (clamped to MIN - MAX range from Constants)
     */
    public void setDriveSpeed(double speed) {
        driveSpeed = Math.max(Constants.MIN_DRIVE_SPEED, Math.min(Constants.MAX_DRIVE_SPEED, speed));
    }

    /**
     * Updates telemetry display with current drive system information.
     * Shows drive mode, speed, pose, IMU heading, and Panels drawing status.
     */
    private void addTelemetry() {
        Pose currentPose = getPose();

        // Drive configuration
        robot.sensors.addTelemetry("═══ Drive Status ═══", "");
        robot.sensors.addTelemetry("Mode", fieldCentric ? "Field-Centric" : "Robot-Centric");
        robot.sensors.addTelemetry("Speed", "%.0f%%", driveSpeed * 100);

        // Localization data
        robot.sensors.addTelemetry("═══ Position ═══", "");
        robot.sensors.addTelemetry("Position", "X:%.1f\" Y:%.1f\"",
                currentPose.getX(), currentPose.getY());
        robot.sensors.addTelemetry("Heading", "Pedro:%.1f° IMU:%.1f°",
                Math.toDegrees(currentPose.getHeading()),
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
    }

    // ============================================================
    //                    GETTERS AND SETTERS

    /**
     * @return Current field-centric mode state
     */
    public boolean isFieldCentric() {
        return fieldCentric;
    }

    /**
     * @return Current drive speed multiplier (0.0 - 1.0)
     */
    public double getDriveSpeed() {
        return driveSpeed;
    }
}