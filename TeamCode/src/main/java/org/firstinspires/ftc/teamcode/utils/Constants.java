package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {

    public static final double KP = 0.2;

    // ----- HARDWARE MAP NAMES ------
    // -------- SENSOR NAMES ---------
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

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .build();
    }
}
