package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.seattlesolvers.solverslib.command.Subsystem;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Ganymede;
import org.firstinspires.ftc.teamcode.utils.Constants;

public class Sensors implements Subsystem {
    private Limelight3A limelight;
    private Telemetry telemetry;
    private double x, y, theta;

    // Flag to enable/disable AprilTag position tracking
    public boolean aprilTagPositionTracking = false;

    // Last detection metrics for debugging
    private long lastDetectionTime = 0;
    private double lastDetectionDistance = 0;
    private int totalDetections = 0;
    private int filteredDetections = 0;
    private int acceptedDetections = 0;

    public Sensors(Ganymede robot) {
        telemetry = robot.telemetry;

        try {
//            limelight = robot.hardwareMap.get(Limelight3A.class, Constants.LIMELIGHT_NAME);
//            limelight.setPollRateHz(100);
//            limelight.start();
//            limelight.pipelineSwitch(0);
        } catch (Exception e) {
            telemetry.addData("Limelight Error", e.getMessage());
        }
    }

    public LLResult getLLResult() {
        return limelight.getLatestResult();
    }

    @Override
    public void periodic() {

        // !!! THIS SHOULD BE THE ONLY TELEMETRY UPDATE IN THE WHOLE PROJECT !!!
        telemetry.update();
    }

}