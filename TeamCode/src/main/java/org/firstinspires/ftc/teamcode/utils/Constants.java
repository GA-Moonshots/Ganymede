package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {

    public static final double KP = 0.2;
    public static final double INPUT_THRESHOLD = 0.1;


    // ----- HARDWARE MAP NAMES ------
    // -------- SENSOR NAMES ---------
    public static final String LIMELIGHT_NAME = "limelight";
    public static final String IMU_NAME = "imu";
    public static final String REAR_DIST_NAME = "rear_distance";
    public static final String RIGHT_DIST_NAME = "right_distance";
    public static final String LEFT_DIST_NAME = "left_distance";

    // --------- MOTOR NAMES ---------
    public static final String LEFT_FRONT_NAME = "leftFront";
    public static final String RIGHT_FRONT_NAME = "rightFront";
    public static final String LEFT_BACK_NAME = "leftBack";
    public static final String RIGHT_BACK_NAME = "rightBack";


    // encoder aliases
    public static final String LEFT_ODOMETRY_NAME = LEFT_FRONT_NAME;
    // TODO: Right odom should be right front
    // center should be right back
    public static final String RIGHT_ODOMETRY_NAME = LEFT_BACK_NAME;
    public static final String CENTER_ODOMETRY_NAME = RIGHT_FRONT_NAME;

    // --------- SERVO NAMES ---------


    // ---- PEDRO PATHING -----
    public static FollowerConstants followerConstants = new FollowerConstants();

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static ThreeWheelIMUConstants localizerConstants = new ThreeWheelIMUConstants()
            .forwardTicksToInches(.001989436789) //need to run tests to figure it out
            .strafeTicksToInches(.001989436789)
            .turnTicksToInches(.001989436789)
            .leftPodY(8.007739252)
            .rightPodY(-7.658540267)
            .strafePodX(-6.370904873)
            .leftEncoder_HardwareMapName(LEFT_FRONT_NAME) // REFERENCE VARIABLE ABOVE. MOTOR NAME, NOT PAR
            .rightEncoder_HardwareMapName(RIGHT_BACK_NAME) // REFERENCE VARIABLE ABOVE. MOTOR NAME, NOT PAR
            .strafeEncoder_HardwareMapName(RIGHT_FRONT_NAME) // REFERENCE VARIABLE ABOVE. MOTOR NAME, NOT PERP
            .leftEncoderDirection(Encoder.FORWARD)  // if we are noticing  crazy # one of these might need to be reversed
            .rightEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.FORWARD)
            .IMU_HardwareMapName(IMU_NAME)
            .IMU_Orientation(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT));

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .threeWheelIMULocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}
