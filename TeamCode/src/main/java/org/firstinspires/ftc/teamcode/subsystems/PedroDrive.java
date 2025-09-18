package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import com.pedropathing.follower.Follower;
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

    /**
     * ┌─────────────────────────────────────────────────────────────────────┐
     * │                         CONSTRUCTOR                                 │
     * │                                                                     │
     * │  Initializes the Pedro Drive subsystem with motor configuration,     │
     * │  IMU setup, and Pedro Pathing follower initialization.              │
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
        // Post basic data
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
    //                    DRIVE CONTROL
    // ============================================================

    /**
     * Main teleop drive method using mecanum kinematics.
     * Supports both field-centric and robot-centric control modes.
     *
     * @param forward Forward/backward input (-1.0 to 1.0)
     * @param strafe  Left/right strafe input (-1.0 to 1.0)
     * @param turn    Rotation input (-1.0 to 1.0)
     */
    public void drive(double forward, double strafe, double turn) {
        double rotY = forward * driveSpeed;
        double rotX = strafe * driveSpeed;
        double rotation = turn * driveSpeed;

        // Field-centric transformation if enabled
        if (fieldCentric) {
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate input vector by robot heading
            double temp = rotY;
            rotY = rotY * Math.cos(-botHeading) - rotX * Math.sin(-botHeading);
            rotX = temp * Math.sin(-botHeading) + rotX * Math.cos(-botHeading);
        };

        // ============ Mecanum Kinematics Calculation ============
        // Calculate individual wheel powers using mecanum equations
        // Denominator ensures no wheel power exceeds [-1, 1] range
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);

        // TODO: Which of these needs to be subtraction?
        double frontLeftPower = (rotY + rotX - turn) / denominator;
        double backLeftPower = (rotY - rotX - turn) / denominator;
        double frontRightPower = (rotY - rotX + turn) / denominator;
        double backRightPower = (rotY + rotX + turn) / denominator;
        
        // Apply calculated powers to motors
        leftFront.setPower(frontLeftPower);
        leftBack.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightBack.setPower(backRightPower);
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
    }

    /**
     * Sets the global drive speed multiplier for precision control.
     *
     * @param speed Desired speed multiplier (clamped to MIN - MAX range from Constants)
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

        robot.telemetry.addData("Saved Pose", "X:%.1f Y:%.1f H:%.1f°",
                currentPose.getX(),
                currentPose.getY(),
                Math.toDegrees(currentPose.getHeading()));
    }

    /**
     * Updates telemetry display with current drive system information.
     * Shows drive mode, speed, pose, and IMU heading for debugging.
     */
    private void addTelemetry() {
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