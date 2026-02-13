package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    /** Drive motor names - must match hardware map exactly */
    public static final String LEFT_FRONT_NAME = "leftFront";
    public static final String RIGHT_FRONT_NAME = "rightFront";
    public static final String LEFT_BACK_NAME = "leftBack";
    public static final String RIGHT_BACK_NAME = "rightBack";

    public static final String LAUNCHER_NAME = "launcher";
    public static final String INTAKE_NAME = "intake";

    /* Servo */
    public static final String COLOR_GATE_NAME = "colorGate";
    public static final String TURRET_SERVO_NAME = "turretServo";
    public static final String LAUNCHER_SERVO_NAME = "launcherServo";
    public static final String GREEN_FEEDER_SERVO_NAME = "greenServo";
    public static final String PURPLE_FEEDER_SERVO_NAME = "purpleServo";
    public static final String GREEN_SORTER_NAME = "stopperG";
    public static final String PURPLE_SORTER_NAME = "stopperP";


    /* Sensors */
    public static final String LEFT_BUTTON_NAME = "leftButton";
    public static final String FRONT_BUTTON_NAME = "frontButton";

    /** Motor directions - verify this first with teleOp driving while propped up on a foam block */
    public static final DcMotorSimple.Direction LEFT_FRONT_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction LEFT_BACK_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction RIGHT_FRONT_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction RIGHT_BACK_DIRECTION = DcMotorSimple.Direction.FORWARD;

    /** Zero power behavior: BRAKE = resist, FLOAT = free spin */
    public static final DcMotor.ZeroPowerBehavior DRIVE_ZERO_POWER_BEHAVIOR =
            DcMotor.ZeroPowerBehavior.BRAKE;

    // ============================================================
    //             MOTOR POWER LIMITS, TOLERANCES, & SETTINGS

    /** Maximum drive power (0.0 to 1.0) */
    public static final double MAX_DRIVE_POWER = 1.0;

    /** Drive speed multipliers for teleop */
    public static final double MIN_DRIVE_SPEED = 0.2;
    public static final double MAX_DRIVE_SPEED = 1.0;
    public static final double DEFAULT_DRIVE_SPEED = 1;
    public static final double SLOW_MODE_MULTIPLIER = 0.5;

    /** Input threshold and tolerances */
    public static final double INPUT_THRESHOLD = 0.1;
    public static final double POSE_TOLERANCE = 0.5;
    public static final double HEADING_TOLERANCE_DEGREES = 10.0;

    /** Default field-centric mode on startup */
    public static final boolean DEFAULT_FIELD_CENTRIC = true;

    /** Servo power for rotation - adjust as needed for your mechanism */
    public static final double TURRET_ROTATION_POWER = 1;

    // ============================================================
    //                    FIELD GEOMETRY & SCORING

    /** Field corner coordinates */
    public static final double FIELD_BOTTOM_LEFT_X = 0;
    public static final double FIELD_BOTTOM_LEFT_Y = -86;
    public static final double FIELD_TOP_RIGHT_X = 230;
    public static final double FIELD_TOP_RIGHT_Y = 144;

    /** Blue alliance scoring target  */
    public static final double BLUE_TARGET_X = 27;
    public static final double BLUE_TARGET_Y = 124;

    /** Red alliance scoring target  */
    public static final double RED_TARGET_X = 207;
    public static final double RED_TARGET_Y = 123;

    /**
     * Launcher heading offset in degrees
     *
     * When the turret is in LEFT position, the launcher is rotated
     * 90 degrees from the robot's heading. To aim at the goal, we
     * need to offset the robot's heading by this amount.
     */
    public static final double LAUNCHER_LEFT_HEADING_OFFSET_DEGREES = 90.0;

    // ============================================================
    //                    LAUNCHER CONSTANTS

    /** Idle flywheel power - keeps motor warm for faster spin-up */
    public static final double LAUNCHER_IDLE_POWER = -0.3;

    /** Default launch power when no specific power is provided */
    public static final double LAUNCHER_DEFAULT_POWER = -0.7;

    /** RPM at which the flywheel is fast enough to start feeding */
    public static final double LAUNCHER_FEED_RPM = 150.0;

    /** RPM drop threshold indicating ball has been launched */
    public static final double LAUNCHER_LAUNCHED_RPM = 135.0;

    /** Safety timeout for RPM-based launch (seconds) */
    public static final double LAUNCHER_RPM_TIMEOUT_SECONDS = 10.0;

    // ============================================================
    //                    SENSOR MAPPINGS

    /** IMU hardware map name */
    public static final String IMU_NAME = "imu";

    public static final String COLOR_SENSOR = "colorSensor";

    public static final String LIMELIGHT_NAME = "limelight";
    /** IMU mounting orientation - update based on REV Hub position */
    public static final RevHubOrientationOnRobot.LogoFacingDirection IMU_LOGO_DIRECTION =
            RevHubOrientationOnRobot.LogoFacingDirection.DOWN;
    public static final RevHubOrientationOnRobot.UsbFacingDirection IMU_USB_DIRECTION =
            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

    // ============================================================
    //                    PEDRO FOLLOWER CONSTANTS

    public static FollowerConstants followerConstants = new FollowerConstants()
            /** Robot mass in kg - weigh robot BEFORE tuning */
            .mass(13.607)

            /** Dual PID flags - start false, enable for boss-level fine tuning (we think) */
            .useSecondaryTranslationalPIDF(false)
            .useSecondaryHeadingPIDF(false)
            .useSecondaryDrivePIDF(false)

            /** Tune with: Automatic → Forward Zero Power Acceleration */
            // https://pedropathing.com/docs/pathing/tuning/automatic#forward-zero-power-acceleration
            .forwardZeroPowerAcceleration(-47.192998)
            /** Tune with: Automatic → Lateral Zero Power Acceleration */
            // https://pedropathing.com/docs/pathing/tuning/automatic#lateral-zero-power-acceleration
            .lateralZeroPowerAcceleration(-88.721913)

            /** Tune with: Manual → Translational - push sideways, adjust until smooth return */
            // https://pedropathing.com/docs/pathing/tuning/pids/translational
            .translationalPIDFCoefficients(new PIDFCoefficients(0.08, 0, 0.003, 0.03))
            //.secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.095, 0, 0.01, 0.03))

            /** Tune with: Manual → Heading - rotate robot, adjust until smooth return */
            // https://pedropathing.com/docs/pathing/tuning/pids/heading
            .headingPIDFCoefficients(new PIDFCoefficients(0.8, 0, 0.02, 0.05))

            /** Tune with: Manual → Drive (optional) - only after other PIDs work */
            // https://pedropathing.com/docs/pathing/tuning/pids/drive
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.001, 0, 0.0, 13, 0.5))

            /** Tune with: Manual → Centripetal - run circles, typical: 0.001-0.01 */
            // https://pedropathing.com/docs/pathing/tuning/pids/centripetal
            .centripetalScaling(0.0001);

    // ============================================================
    //               MECANUM DRIVETRAIN CONSTANTS

    public static MecanumConstants mecanumConstants = new MecanumConstants()
            /** Tune with: Automatic → Forward Velocity  */
            // https://pedropathing.com/docs/pathing/tuning/automatic#forward-velocity-tuner
            .xVelocity(69.96109)
            /** Tune with: Automatic → Lateral Velocity */
            .yVelocity(54.53147)
            .maxPower(MAX_DRIVE_POWER)
            .rightRearMotorName(RIGHT_BACK_NAME)
            .rightFrontMotorName(RIGHT_FRONT_NAME)
            .leftRearMotorName(LEFT_BACK_NAME)
            .leftFrontMotorName(LEFT_FRONT_NAME)
            .leftFrontMotorDirection(LEFT_FRONT_DIRECTION)
            .leftRearMotorDirection(LEFT_BACK_DIRECTION)
            .rightFrontMotorDirection(RIGHT_FRONT_DIRECTION)
            .rightRearMotorDirection(RIGHT_BACK_DIRECTION);

    /**
     * Path completion criteria
     * Adjust after basic tuning complete
     */
    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,   // End T-value (increase if stopping early)
            100,    // Timeout ms (increase if cutting corners)
            1,      // Position tolerance inches
            1       // Heading tolerance radians
    );

    // ============================================================
    //                    PINPOINT LOCALIZATION

    public static PinpointConstants localizerConstants = new PinpointConstants()
            /** Pod offsets from robot center in inches */
            .forwardPodY(3)  // Negative = behind center
            .strafePodX(-9)     // Negative = left of center
            .distanceUnit(DistanceUnit.INCH)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD)
            /** Test directions with Localization Test - forward should increase X */
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            // https://pedropathing.com/docs/pathing/tuning/localization/pinpoint#encoder-directions
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    // ============================================================
    //    FOLLOWER FACTORY - WHERE WE PUT ALL THE STUFF TOGETHER

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(mecanumConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}