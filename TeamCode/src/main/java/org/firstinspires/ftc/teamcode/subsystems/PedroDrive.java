package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Ganymede;
import org.firstinspires.ftc.teamcode.utils.Constants;

/**
 * ╔═══════════════════════════════════════════════════════════════════════════╗
 * ║                        PEDRO DRIVE SUBSYSTEM                              ║
 * ║                                                                           ║
 * ║  Advanced mecanum drive subsystem using Pedro Pathing for precise         ║
 * ║  autonomous path following and smooth teleop control.                     ║
 * ║                                                                           ║
 * ║  Features:                                                                ║
 * ║    • Field-centric and robot-centric drive modes                          ║
 * ║    • Integrated Pedro Pathing localization                                ║
 * ║    • Variable speed control for precision movements                       ║
 * ║    • IMU-based heading correction                                         ║
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


    /**
     * ┌─────────────────────────────────────────────────────────────────────┐
     * │                         CONSTRUCTOR                                 │
     * │                                                                     │
     * │  Initializes the Pedro Drive subsystem with motor configuration,     │
     * │  IMU setup, and Pedro Pathing follower initialization.              │
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
        // Set motor directions for proper mecanum kinematics
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        // Configure zero power behavior for smooth stops
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
    }

    // ============================================================
    //                    PERIODIC UPDATES
    // ============================================================

    /**
     * Called automatically by the command scheduler every loop iteration.
     * Updates Pedro Pathing's localization system.
     */
    @Override
    public void periodic() {
        // Update Pedro's localization system. DO NOT DUPLICATE THIS CALL
        update();
    }

    /**
     * Updates Pedro Pathing's internal localization calculations.
     * This method processes encoder readings and updates the robot's
     * estimated position on the field.
     *
     * Note: This is called automatically in periodic() but can also
     * be called manually if needed for more frequent updates.
     */
    public void update() {
        follower.update();
    }

    // ============================================================
    //                    DRIVE CONTROL
    // ============================================================

    /**
     * Primary drive method for TeleOp control.
     * Implements mecanum kinematics with optional field-centric control.
     *
     * @param forward Forward/backward motion (-1.0 to 1.0, negative = backward)
     * @param strafe  Left/right motion (-1.0 to 1.0, negative = left)
     * @param turn    Rotational motion (-1.0 to 1.0, negative = CCW)
     *
     * Field-Centric Mode:
     *   - Movement is relative to the field (driver's perspective)
     *   - Forward always moves away from driver regardless of robot orientation
     *
     * Robot-Centric Mode:
     *   - Movement is relative to the robot
     *   - Forward moves in the direction the robot is facing
     */
    public void drive(double forward, double strafe, double turn) {
        // Get current robot heading for field-centric calculations
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Initialize movement vectors
        double rotX = strafe;
        double rotY = -forward; // Inverted because gamepad Y is reversed

        // Apply field-centric transformation if enabled
        if (fieldCentric) {
            // Rotate input vector by negative robot heading to align with field
            double temp = rotY * Math.cos(-botHeading) - rotX * Math.sin(-botHeading);
            rotX = rotY * Math.sin(-botHeading) + rotX * Math.cos(-botHeading);
            rotY = temp;
        }

        // Apply global speed modifier for precision control
        rotX *= driveSpeed;
        rotY *= driveSpeed;
        turn *= driveSpeed;

        // ============ Mecanum Kinematics Calculation ============
        // Calculate individual wheel powers using mecanum equations
        // Denominator ensures no wheel power exceeds [-1, 1] range
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);

        double frontLeftPower = (rotY + rotX + turn) / denominator;
        double backLeftPower = (rotY - rotX + turn) / denominator;
        double frontRightPower = (rotY - rotX - turn) / denominator;
        double backRightPower = (rotY + rotX - turn) / denominator;

        // Apply calculated powers to motors
        leftFront.setPower(frontLeftPower);
        leftBack.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightBack.setPower(backRightPower);

        // Update telemetry with current drive state
        updateTelemetry();
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
    //                    LOCALIZATION
    // ============================================================

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

    // ============================================================
    //                    CONFIGURATION METHODS
    // ============================================================

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
        robot.telemetry.addData("Drive Mode", fieldCentric ? "Field-Centric" : "Robot-Centric");
    }

    /**
     * Sets the global drive speed multiplier for precision control.
     *
     * @param speed Desired speed multiplier (clamped to 0.1 - 1.0)
     */
    public void setDriveSpeed(double speed) {
        driveSpeed = Math.max(Constants.MIN_DRIVE_SPEED, Math.min(Constants.MAX_DRIVE_SPEED, speed));
    }

    // ============================================================
    //                    UTILITY METHODS
    // ============================================================

    /**
     * Saves the current robot pose for persistence between OpMode runs.
     * This allows maintaining position across multiple autonomous/teleop sessions.
     *
     * TODO: Implement actual persistence using SharedPreferences or file I/O
     */
    public void saveCurrentPose() {
        Pose currentPose = getPose();
        // TODO: Implement actual pose persistence
        // Options:
        //   1. SharedPreferences for simple x,y,heading storage
        //   2. File I/O for more complex data
        //   3. Network storage for multi-device access

        robot.telemetry.addData("Saved Pose", "X:%.1f Y:%.1f H:%.1f°",
                currentPose.getX(),
                currentPose.getY(),
                Math.toDegrees(currentPose.getHeading()));
    }

    /**
     * Updates telemetry display with current drive system information.
     * Shows drive mode, speed, pose, and IMU heading for debugging.
     */
    private void updateTelemetry() {
        Pose currentPose = getPose();

        // Drive configuration
        robot.telemetry.addData("═══ Drive Status ═══", "");
        robot.telemetry.addData("Mode", fieldCentric ? "Field-Centric" : "Robot-Centric");
        robot.telemetry.addData("Speed", "%.0f%%", driveSpeed * 100);

        // Localization data
        robot.telemetry.addData("═══ Position ═══", "");
        robot.telemetry.addData("Position", "X:%.1f\" Y:%.1f\"",
                currentPose.getX(), currentPose.getY());
        robot.telemetry.addData("Heading", "Pedro:%.1f° IMU:%.1f°",
                Math.toDegrees(currentPose.getHeading()),
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
    }

    // ============================================================
    //                    GETTERS AND SETTERS
    // ============================================================

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