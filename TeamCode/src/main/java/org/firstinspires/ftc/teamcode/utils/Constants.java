package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * ╔═══════════════════════════════════════════════════════════════════════════╗
 * ║                         ROBOT CONSTANTS CENTRAL                           ║
 * ║                   Where all the magic numbers live! ✨                    ║
 * ║                   ✨ Settings that change every year                      ║
 * ╚═══════════════════════════════════════════════════════════════════════════╝
 */
public class Constants {

    /*
     * ┌─────────────────────────────────────────────────────────────────────┐
     * │                        DRIVE CONSTANTS                              │
     * │                    ⚙️ Driving & Control ⚙️                          │
     * └─────────────────────────────────────────────────────────────────────┘
     */

    /** Proportional gain for drive control (0.0 - 1.0) */
    public static final double KP = 0.2;

    /** Minimum joystick input threshold to prevent drift */
    public static final double INPUT_THRESHOLD = 0.1;

    /** Maximum drive power multiplier for safety (0.0 - 1.0) */
    public static final double MAX_DRIVE_POWER = 1.0;

    /** Minimum drive speed multiplier (prevents robot from being too slow) */
    public static final double MIN_DRIVE_SPEED = 0.1;

    /** Maximum drive speed multiplier (safety limit) */
    public static final double MAX_DRIVE_SPEED = 1.0;

    /** Default drive speed on initialization (1.0 = full speed) */
    public static final double DEFAULT_DRIVE_SPEED = 1.0;

    /** Default field-centric mode on initialization */
    public static final boolean DEFAULT_FIELD_CENTRIC = true;

    /** Slow mode speed multiplier when activated */
    public static final double SLOW_MODE_MULTIPLIER = 0.25;


    /*
     * ╔═══════════════════════════════════════════════════════════════════╗
     * ║                      HARDWARE MAP NAMES                           ║
     * ║                    🤖 Robot Configuration 🤖                       ║
     * ╚═══════════════════════════════════════════════════════════════════╝
     */

    // ============================================================
    //                    🎮 MOTOR MAPPINGS 🎮
    // ============================================================

    /** Drive motor hardware map names - must match robot configuration */
    public static final String LEFT_FRONT_NAME = "leftFront";
    public static final String RIGHT_FRONT_NAME = "rightFront";
    public static final String LEFT_BACK_NAME = "leftBack";
    public static final String RIGHT_BACK_NAME = "rightBack";

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
     *
     * TUNING GUIDE:
     * - translationalPID: Controls forward/backward accuracy
     * - headingPID: Controls rotational accuracy
     * - drivePID: Controls overall path following smoothness
     */
    public static FollowerConstants followerConstants = new FollowerConstants()
            // Translational PID - controls forward/backward movement accuracy
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0, 0))

            // Heading PID - controls rotational accuracy
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0, 0))

            // Drive PID - controls path following accuracy
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.025, 0, 0.00001, 0.6, 0))

            // Robot mass in kilograms (for centripetal force compensation)
            .mass(5.0);


    /**
     * Mecanum drivetrain configuration
     * Defines motor ports, directions, and power limits
     */
    public static MecanumConstants mecanumConstants = new MecanumConstants()
            .maxPower(MAX_DRIVE_POWER)
            .rightRearMotorName(RIGHT_BACK_NAME)
            .rightFrontMotorName(RIGHT_FRONT_NAME)
            .leftRearMotorName(LEFT_BACK_NAME)
            .leftFrontMotorName(LEFT_FRONT_NAME)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

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
     * ⚠️ IMPORTANT: Use the Pedro Tuning OpMode to find these values
     */
    public static ThreeWheelIMUConstants localizerConstants = new ThreeWheelIMUConstants()
            // ============ ENCODER CONVERSION FACTORS ============
            // These convert encoder ticks to inches - MUST BE TUNED!
            .forwardTicksToInches(0.001989436789)  // From forward tuner
            .strafeTicksToInches(0.001989436789)   // From lateral tuner
            .turnTicksToInches(0.001989436789)     // From turn tuner

            // ============ PHYSICAL OFFSETS ============
            // Pod positions relative to robot center (inches)
            .leftPodY(1)        // Left encoder Y offset
            .rightPodY(-1)      // Right encoder Y offset
            .strafePodX(-2.5)   // Strafe encoder X offset

            // ============ HARDWARE CONFIGURATION ============
            // Encoder port names (use motor ports)
            .leftEncoder_HardwareMapName(LEFT_FRONT_NAME) // TODO: verify these are correct
            .rightEncoder_HardwareMapName(RIGHT_BACK_NAME)
            .strafeEncoder_HardwareMapName(RIGHT_FRONT_NAME)

            // Encoder directions (adjust if needed)
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.FORWARD)

            // IMU configuration
            .IMU_HardwareMapName(IMU_NAME)
            .IMU_Orientation(new RevHubOrientationOnRobot(
                    IMU_LOGO_DIRECTION,
                    IMU_USB_DIRECTION
            ));


    /*
     * ╔═══════════════════════════════════════════════════════════════════╗
     * ║                      BUILDER METHOD                               ║
     * ║                Creates configured Pedro Follower                   ║
     * ╚═══════════════════════════════════════════════════════════════════╝
     */

    /**
     * Factory method to create a configured Pedro Pathing Follower
     * This method assembles all the constants into a working Follower instance
     *
     * @param hardwareMap Hardware map from OpMode
     * @return Configured Follower ready for path following
     */
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(mecanumConstants)
                .threeWheelIMULocalizer(localizerConstants)
                .build();
    }
}