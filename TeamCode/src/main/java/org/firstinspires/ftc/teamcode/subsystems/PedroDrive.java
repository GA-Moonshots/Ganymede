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
 * Drive subsystem using Pedro Pathing for advanced path following
 * Replaces the old RoadRunner-based Mecanum drive
 */
public class PedroDrive extends SubsystemBase {

    // Robot reference
    private final Ganymede robot;

    // Pedro Pathing Follower - handles all path following and localization
    public final Follower follower;

    // Drive motors
    private final DcMotorEx leftFront, leftBack, rightFront, rightBack;

    // IMU for heading
    private final IMU imu;

    // Drive configuration
    private boolean fieldCentric = true;
    private double driveSpeed = 1.0; // Speed multiplier


    /**
     * Initialize the Pedro Pathing drive system
     */
    public PedroDrive(Ganymede robot, Pose startPose) {
        this.robot = robot;

        // Initialize motors
        leftFront = robot.hardwareMap.get(DcMotorEx.class, Constants.LEFT_FRONT_NAME);
        leftBack = robot.hardwareMap.get(DcMotorEx.class, Constants.LEFT_BACK_NAME);
        rightFront = robot.hardwareMap.get(DcMotorEx.class, Constants.RIGHT_FRONT_NAME);
        rightBack = robot.hardwareMap.get(DcMotorEx.class, Constants.RIGHT_BACK_NAME);

        // Configure motors
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set zero power behavior
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize IMU
        imu = robot.hardwareMap.get(IMU.class, Constants.IMU_NAME);
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(parameters);

        // Initialize Pedro Pathing Follower
        follower = Constants.createFollower(robot.hardwareMap);
        follower.setStartingPose(startPose);
    }

    /**
     * Main drive method for TeleOp - handles field-centric mecanum drive
     * @param forward Forward/backward motion (-1 to 1)
     * @param strafe Left/right motion (-1 to 1)
     * @param turn Rotation (-1 to 1)
     */
    public void drive(double forward, double strafe, double turn) {
        // Get current heading for field-centric calculations
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Field-centric transformation
        double rotX = strafe;
        double rotY = -forward; // Negative because gamepad Y is inverted

        if (fieldCentric) {
            // Rotate input vector by robot heading
            double temp = rotY * Math.cos(-botHeading) - rotX * Math.sin(-botHeading);
            rotX = rotY * Math.sin(-botHeading) + rotX * Math.cos(-botHeading);
            rotY = temp;
        }

        // Apply speed modifier
        rotX *= driveSpeed;
        rotY *= driveSpeed;
        turn *= driveSpeed;

        // Calculate wheel powers (mecanum kinematics)
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);
        double frontLeftPower = (rotY + rotX + turn) / denominator;
        double backLeftPower = (rotY - rotX + turn) / denominator;
        double frontRightPower = (rotY - rotX - turn) / denominator;
        double backRightPower = (rotY + rotX - turn) / denominator;

        // Set motor powers
        leftFront.setPower(frontLeftPower);
        leftBack.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightBack.setPower(backRightPower);

        // Update telemetry
        updateTelemetry();
    }

    /**
     * Update Pedro's localization
     * Should be called every loop iteration
     */
    public void update() {
        follower.update();
    }


    /**
     * Stop all drive motors
     */
    public void stop() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    /**
     * Get current robot pose from Pedro's localization
     */
    public Pose getPose() {
        return follower.getPose();
    }

    /**
     * Set the robot's pose (use for relocalization with April Tags)
     */
    public void setPose(Pose newPose) {
        follower.setPose(newPose);
    }

    /**
     * Reset heading to 0 (facing forward)
     */
    public void resetHeading() {
        imu.resetYaw();
        // Also reset Pedro's heading
        Pose currentPose = getPose();
        setPose(new Pose(currentPose.getX(), currentPose.getY(), 0));
    }

    /**
     * Toggle field-centric driving mode
     */
    public void toggleFieldCentric() {
        fieldCentric = !fieldCentric;
        robot.telemetry.addData("Drive Mode", fieldCentric ? "Field-Centric" : "Robot-Centric");
    }

    /**
     * Set drive speed multiplier
     */
    public void setDriveSpeed(double speed) {
        driveSpeed = Math.max(0.1, Math.min(1.0, speed));
    }


    /**
     * Save current pose for persistence between runs
     */
    public void saveCurrentPose() {
        // Implement your pose persistence logic here
        // Could save to a file or SharedPreferences
        Pose currentPose = getPose();
        robot.telemetry.addData("Saved Pose", "X:%.1f Y:%.1f H:%.1f",
                currentPose.getX(), currentPose.getY(), currentPose.getHeading());
    }

    /**
     * Update telemetry with drive information
     */
    private void updateTelemetry() {
        Pose currentPose = getPose();
        robot.telemetry.addData("Drive Mode", fieldCentric ? "Field-Centric" : "Robot-Centric");
        robot.telemetry.addData("Speed", "%.1f%%", driveSpeed * 100);
        robot.telemetry.addData("Pose", "X:%.1f Y:%.1f H:%.1f",
                currentPose.getX(), currentPose.getY(), currentPose.getHeading());
        robot.telemetry.addData("IMU Heading", "%.1f deg",
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
    }

    @Override
    public void periodic() {
        // This runs every scheduler loop
        // Update Pedro's localization
        update();
    }
}