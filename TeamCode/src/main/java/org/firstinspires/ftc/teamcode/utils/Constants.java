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


    /*
     * ╔═══════════════════════════════════════════════════════════════════╗
     * ║                      HARDWARE MAP NAMES                           ║
     * ║                    🤖 Robot Configuration 🤖                      ║
     * ╚═══════════════════════════════════════════════════════════════════╝
     */

    // ============================================================
    //                    🎮 MOTOR MAPPINGS 🎮
    // ============================================================

    /** Left front drive motor - Control Hub Port 0 */
    public static final String LEFT_FRONT_NAME = "leftFront";

    /** Right front drive motor - Control Hub Port 1 */
    public static final String RIGHT_FRONT_NAME = "rightFront";

    /** Left back drive motor - Control Hub Port 2 */
    public static final String LEFT_BACK_NAME = "leftBack";

    /** Right back drive motor - Control Hub Port 3 */
    public static final String RIGHT_BACK_NAME = "rightBack";


    // ============================================================
    //                    📡 SENSOR MAPPINGS 📡
    // ============================================================

    /** Limelight 3A vision camera */
    public static final String LIMELIGHT_NAME = "limelight";

    /** REV IMU for heading data */
    public static final String IMU_NAME = "imu";


    // ============================================================
    //                 🎯 ODOMETRY ENCODER MAPPINGS 🎯
    // ============================================================

    /**
     * Odometry encoder port aliases - these map to the motor ports
     * but are used by the odometry system for position tracking
     *
     */

    /** Left odometry pod - shares port with left front motor */
    public static final String LEFT_ODOMETRY_NAME = LEFT_FRONT_NAME;

    /** Right odometry pod - shares port with right front motor */
    public static final String RIGHT_ODOMETRY_NAME = RIGHT_FRONT_NAME;

    /** Center (perpendicular) odometry pod - shares port with right back motor */
    public static final String CENTER_ODOMETRY_NAME = RIGHT_BACK_NAME;


    // ============================================================
    //                    🎭 SERVO MAPPINGS 🎭
    // ============================================================

    // TODO: Add servo mappings when mechanisms are installed
    // Example:
    // public static final String CLAW_SERVO_NAME = "claw";
    // public static final String ARM_SERVO_NAME = "arm";


    /*
     * ████████████████████████████████████████████████████████████████████
     * █                                                                  █
     * █                     PEDRO PATHING CONFIGURATION                  █
     * █                         🗺️ Path Following 🗺️                     █
     * █                                                                  █
     * ████████████████████████████████████████████████████████████████████
     */

    /**
     * Pedro Pathing follower constants
     * Contains PID values and path following parameters
     * Tune these values using the Pedro Tuning OpMode!
     */
    public static FollowerConstants followerConstants = new FollowerConstants()
            // Translational PID - controls forward/backward movement accuracy
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0, 0))

            // Heading PID - controls rotational accuracy
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0, 0))

            // Drive PID - controls path following accuracy
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.025, 0, 0.00001, 0.6, 0)

            // Optional: Enable dual PID for better correction
            // .useSecondaryTranslationalPIDF(false)
            // .useSecondaryHeadingPIDF(false)
            // .useSecondaryDrivePIDF(false)
            );

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
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
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
     *
     * IMPORTANT: These values MUST be tuned for your specific robot!
     * Use the Pedro Tuning OpMode to find these values
     */
    public static ThreeWheelIMUConstants localizerConstants = new ThreeWheelIMUConstants()
            // Encoder tick-to-inch conversion factors (MUST BE TUNED!)
            .forwardTicksToInches(0.001989436789)  // Tune with Forward Localizer Test
            .strafeTicksToInches(0.001989436789)   // Tune with Strafe Localizer Test
            .turnTicksToInches(0.001989436789)     // Tune with Turn Localizer Test

            // Physical pod positions from robot center (inches)
            .leftPodY(8.007739252)     // Distance from center to left pod
            .rightPodY(-7.658540267)   // Distance from center to right pod (negative = right)
            .strafePodX(-6.370904873)  // Distance from center to perpendicular pod (negative = back)

            // Encoder hardware mappings
            .leftEncoder_HardwareMapName(LEFT_ODOMETRY_NAME)
            .rightEncoder_HardwareMapName(RIGHT_ODOMETRY_NAME)
            .strafeEncoder_HardwareMapName(CENTER_ODOMETRY_NAME)

            // Encoder directions (adjust if counting backwards)
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.FORWARD)

            // IMU configuration
            .IMU_HardwareMapName(IMU_NAME)
            .IMU_Orientation(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
            ));


    /*
     * ╭───────────────────────────────────────────────────────────────╮
     * │                    FACTORY METHODS                            │
     * │                  🏭 Object Builders 🏭                        │
     * ╰───────────────────────────────────────────────────────────────╯
     */

    /**
     * Creates and configures a Pedro Pathing Follower instance
     * This is the main entry point for autonomous path following
     *
     * @param hardwareMap The OpMode's hardware map
     * @return Configured Follower ready for path execution
     */
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(mecanumConstants)
                .threeWheelIMULocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }


    /*
     * ╔═══════════════════════════════════════════════════════════════╗
     * ║                    UTILITY CONSTANTS                          ║
     * ║                   🛠️ Helper Values 🛠️                         ║
     * ╚═══════════════════════════════════════════════════════════════╝
     */

    /** Field dimensions for path planning (inches) */
    public static final double FIELD_SIZE = 144.0;  // 12 feet square

    /** Robot dimensions (inches) */
    public static final double ROBOT_WIDTH = 18.0;
    public static final double ROBOT_LENGTH = 18.0;

    /** Starting positions for autonomous */
    public static class StartingPositions {
        // Blue Alliance
        public static final double BLUE_LEFT_X = 12.0;
        public static final double BLUE_LEFT_Y = 63.0;
        public static final double BLUE_RIGHT_X = 36.0;
        public static final double BLUE_RIGHT_Y = 63.0;

        // Red Alliance
        public static final double RED_LEFT_X = -36.0;
        public static final double RED_LEFT_Y = 63.0;
        public static final double RED_RIGHT_X = -12.0;
        public static final double RED_RIGHT_Y = 63.0;
    }


    /*
     * ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░
     * ░                                                              ░
     * ░                    END OF CONSTANTS FILE                     ░
     * ░             May the pizza be delivered on time 🍀            ░
     * ░                                                              ░
     * ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░
     */
}