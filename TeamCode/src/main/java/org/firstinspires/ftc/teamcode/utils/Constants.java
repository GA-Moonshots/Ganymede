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

/**
 * Pedro Pathing Configuration
 *
 * TUNING ORDER (use Tuning OpMode):
 * 1. Setup: Configure hardware mappings below
 * 2. Localization: Test → Forward → Lateral → Turn Tuners
 * 3. Automatic: Forward Velocity → Lateral Velocity → Zero Power Tuners
 * 4. Manual PIDs: Translational → Heading → Drive (optional)
 * 5. Centripetal: For high-speed curves
 */
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
    public static final String STOPER_SERVO_NAME = "stoperServo";


    /* Sensors */
    public static final String LEFT_BUTTON_NAME = "leftButton";
    public static final String FRONT_BUTTON_NAME = "frontButton";

    /** Motor directions - verify this first with teleOp driving while propped up on a foam block */
    public static final DcMotorSimple.Direction LEFT_FRONT_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction LEFT_BACK_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction RIGHT_FRONT_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction RIGHT_BACK_DIRECTION = DcMotorSimple.Direction.REVERSE;

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
    public static final double DEFAULT_DRIVE_SPEED = 0.8;
    public static final double SLOW_MODE_MULTIPLIER = 0.5;

    /** Input threshold and tolerances */
    public static final double INPUT_THRESHOLD = 0.1;
    public static final double POSE_TOLERANCE = 0.5;

    /** Default field-centric mode on startup */
    public static final boolean DEFAULT_FIELD_CENTRIC = true;

    /** Servo power for rotation - adjust as needed for your mechanism */
    public static final double TURRET_ROTATION_POWER = 0.5;

    // ============================================================
    //                    SENSOR MAPPINGS

    /** IMU hardware map name */
    public static final String IMU_NAME = "imu";

    public static final String COLOR_SENSOR = "sensor_color_distance";

    /** IMU mounting orientation - update based on REV Hub position */
    public static final RevHubOrientationOnRobot.LogoFacingDirection IMU_LOGO_DIRECTION =
            RevHubOrientationOnRobot.LogoFacingDirection.DOWN;
    public static final RevHubOrientationOnRobot.UsbFacingDirection IMU_USB_DIRECTION =
            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

    // ============================================================
    //                    PEDRO FOLLOWER CONSTANTS

    public static FollowerConstants followerConstants = new FollowerConstants()
            /** Robot mass in kg - weigh robot BEFORE tuning */
            .mass(4.99)

            /** Dual PID flags - start false, enable for boss-level fine tuning (we think) */
            .useSecondaryTranslationalPIDF(false)
            .useSecondaryHeadingPIDF(false)
            .useSecondaryDrivePIDF(false)

            /** Tune with: Automatic → Forward Zero Power Acceleration */
            // https://pedropathing.com/docs/pathing/tuning/automatic#forward-zero-power-acceleration
            .forwardZeroPowerAcceleration(-65.89931154311324)
            /** Tune with: Automatic → Lateral Zero Power Acceleration */
            // https://pedropathing.com/docs/pathing/tuning/automatic#lateral-zero-power-acceleration
            .lateralZeroPowerAcceleration(-65.69922401646974)

            /** Tune with: Manual → Translational - push sideways, adjust until smooth return */
            // https://pedropathing.com/docs/pathing/tuning/pids/translational
            .translationalPIDFCoefficients(new PIDFCoefficients(0.095, 0, 0.01, 0.03))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.095, 0, 0.01, 0.03))

            /** Tune with: Manual → Heading - rotate robot, adjust until smooth return */
            // https://pedropathing.com/docs/pathing/tuning/pids/heading
            .headingPIDFCoefficients(new PIDFCoefficients(0.0, 0, 0.0, 0))

            /** Tune with: Manual → Drive (optional) - only after other PIDs work */
            // https://pedropathing.com/docs/pathing/tuning/pids/drive
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.0, 0, 0.00, 0.0, 0.5))

            /** Tune with: Manual → Centripetal - run circles, typical: 0.001-0.01 */
            // https://pedropathing.com/docs/pathing/tuning/pids/centripetal
            .centripetalScaling(0.005);

    // ============================================================
    //               MECANUM DRIVETRAIN CONSTANTS

    public static MecanumConstants mecanumConstants = new MecanumConstants()
            /** Tune with: Automatic → Forward Velocity  */
            // https://pedropathing.com/docs/pathing/tuning/automatic#forward-velocity-tuner
            .xVelocity(44.012101301058074)
            /** Tune with: Automatic → Lateral Velocity */
            .yVelocity(38.33342712882936)
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
            .forwardPodY(-3.5)  // Negative = behind center
            .strafePodX(-4)     // Negative = left of center
            .distanceUnit(DistanceUnit.INCH)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD)
            /** Test directions with Localization Test - forward should increase X */
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            // https://pedropathing.com/docs/pathing/tuning/localization/pinpoint#encoder-directions
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

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