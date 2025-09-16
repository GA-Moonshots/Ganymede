package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
public class Constants {

    /*
     * ╔═══════════════════════════════════════════════════════════════════╗
     * ║                    DRIVE MOTOR CONFIGURATION                      ║
     * ║                  🔧 Machine-Specific Settings 🔧                  ║
     * ╚═══════════════════════════════════════════════════════════════════╝
     */

    // ============================================================
    //                    🎮 MOTOR HARDWARE MAPPINGS 🎮
    // ============================================================

    /** Drive motor hardware map names - must match robot configuration */
    public static final String LEFT_FRONT_NAME = "leftFront";
    public static final String RIGHT_FRONT_NAME = "rightFront";
    public static final String LEFT_BACK_NAME = "leftBack";
    public static final String RIGHT_BACK_NAME = "rightBack";

    // ============================================================
    //                    ⚙️ MOTOR DIRECTIONS ⚙️
    // ============================================================

    /**
     * Motor direction configuration for proper mecanum kinematics
     *
     * IMPORTANT: These are the actual directions used in PedroDrive.java
     * Adjust these based on your specific robot's wiring and gearing
     */
    public static final DcMotorSimple.Direction LEFT_FRONT_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction LEFT_BACK_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction RIGHT_FRONT_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction RIGHT_BACK_DIRECTION = DcMotorSimple.Direction.FORWARD;

    // ============================================================
    //                    🛑 MOTOR BEHAVIOR SETTINGS 🛑
    // ============================================================

    /**
     * Zero power behavior for all drive motors
     * BRAKE: Motors actively resist movement when power is 0
     * FLOAT: Motors spin freely when power is 0
     */
    public static final DcMotor.ZeroPowerBehavior DRIVE_ZERO_POWER_BEHAVIOR =
            DcMotor.ZeroPowerBehavior.BRAKE;

    // ============================================================
    //                    🏎️ MOTOR POWER LIMITS 🏎️
    // ============================================================

    /** Maximum power allowed for drive motors (0.0 to 1.0) */
    public static final double MAX_DRIVE_POWER = 1.0;

    /** Minimum drive speed multiplier for fine control */
    public static final double MIN_DRIVE_SPEED = 0.2;

    /** Maximum drive speed multiplier */
    public static final double MAX_DRIVE_SPEED = 1.0;

    /** Default drive speed multiplier for normal operation */
    public static final double DEFAULT_DRIVE_SPEED = 0.8;

    public static final double SLOW_MODE_MULTIPLIER = 0.25;

    public static final double INPUT_THRESHOLD = 0.1;

    // ============================================================
    //                    🎯 DRIVE MODE DEFAULTS 🎯
    // ============================================================

    /** Default field-centric mode on startup */
    public static final boolean DEFAULT_FIELD_CENTRIC = true;

    // ============================================================
    //                    📡 SENSOR MAPPINGS 📡
    // ============================================================

    /** IMU hardware map name */
    public static final String IMU_NAME = "imu";

    /** IMU mounting orientation on robot */
    public static final RevHubOrientationOnRobot.LogoFacingDirection IMU_LOGO_DIRECTION =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static final RevHubOrientationOnRobot.UsbFacingDirection IMU_USB_DIRECTION =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    /*
     * ╔═══════════════════════════════════════════════════════════════════╗
     * ║                    PEDRO PATHING CONFIGURATION                    ║
     * ║                    🗺️ Path Following Setup 🗺️                     ║
     * ╚═══════════════════════════════════════════════════════════════════╝
     */

    /**
     * Pedro Pathing Follower constants
     * Controls path following behavior and correction
     */
    public static FollowerConstants followerConstants = new FollowerConstants()
            // Translational PID - controls forward/backward accuracy
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0, 0))

            // Heading PID - controls rotational accuracy
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0, 0))

            // Drive PID - controls path following accuracy
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.025, 0, 0.00001, 0.6, 0))

            // Robot mass in kilograms (for centripetal force compensation)
            .mass(5.0);

    /**
     * Mecanum drivetrain configuration for Pedro Pathing
     *
     * NOTE: These directions should match the actual motor directions above
     * They are used by Pedro Pathing's internal motor control
     */
    public static MecanumConstants mecanumConstants = new MecanumConstants()
            .maxPower(MAX_DRIVE_POWER)
            .rightRearMotorName(RIGHT_BACK_NAME)
            .rightFrontMotorName(RIGHT_FRONT_NAME)
            .leftRearMotorName(LEFT_BACK_NAME)
            .leftFrontMotorName(LEFT_FRONT_NAME)
            // Use the same directions as defined above for consistency
            .leftFrontMotorDirection(LEFT_FRONT_DIRECTION)
            .leftRearMotorDirection(LEFT_BACK_DIRECTION)
            .rightFrontMotorDirection(RIGHT_FRONT_DIRECTION)
            .rightRearMotorDirection(RIGHT_BACK_DIRECTION);

    /**
     * Path motion constraints
     * Limits for autonomous path execution
     */
    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,   // Path end T-value (0-1, how close to end)
            100,    // Timeout in milliseconds
            1,      // Translational tolerance (inches)
            1       // Heading tolerance (radians)
    );

    /**
     * Three-wheel odometry with IMU configuration
     * Contains physical measurements and conversion factors
     */
    public static ThreeWheelIMUConstants localizerConstants = new ThreeWheelIMUConstants()
            // ============ ENCODER CONVERSION FACTORS ============
            // These convert encoder ticks to inches - MUST BE TUNED!
            .forwardTicksToInches(0.001989436789)
            .strafeTicksToInches(0.001989436789)
            .turnTicksToInches(0.001989436789)

            // ============ ODOMETRY POD POSITIONS ============
            // Physical pod offsets from robot center (inches)
            .leftPodY(1)
            .rightPodY(-1)
            .strafePodX(-2.5)

            // ============ ENCODER HARDWARE MAPPING ============
            // Map to motor ports (odometry pods share motor ports)
            .leftEncoder_HardwareMapName(LEFT_FRONT_NAME)
            .rightEncoder_HardwareMapName(RIGHT_BACK_NAME)
            .strafeEncoder_HardwareMapName(RIGHT_FRONT_NAME)

            // ============ ENCODER DIRECTIONS ============
            // Set based on physical mounting
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.FORWARD);

    /**
     * Factory method to create a configured Pedro Pathing Follower
     * Centralizes all follower configuration
     */
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(mecanumConstants)
                .threeWheelIMULocalizer(localizerConstants)
                .build();
    }
}