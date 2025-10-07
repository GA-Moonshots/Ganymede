package org.firstinspires.ftc.teamcode.subsystems;

import android.telecom.TelecomManager;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Ganymede;
import org.firstinspires.ftc.teamcode.utils.Constants;

public class Sensors extends SubsystemBase {
    private Limelight3A limelight;
    private Telemetry telemetry;
    private TelemetryManager telemetryM;
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
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        try {
//            limelight = robot.hardwareMap.get(Limelight3A.class, Constants.LIMELIGHT_NAME);
//            limelight.setPollRateHz(100);
//            limelight.start();
//            limelight.pipelineSwitch(0);
        } catch (Exception e) {
            addTelemetry("Limelight Error", e.getMessage());
        }
    }

    public LLResult getLLResult() {
        return limelight.getLatestResult();
    }

    @Override
    public void periodic() {
        // !!! THIS SHOULD BE THE ONLY TELEMETRY UPDATE IN THE WHOLE PROJECT !!!
        telemetryM.update(telemetry);
    }

    public void addTelemetry(String key, String value){
        //telemetry.addData(key, value);
        telemetryM.addData(key, value);
    }

    public void addTelemetry(String key, String format, Object... args) {
        //telemetry.addData(key, format, args);
        String formattedValue = String.format(format, args); // Manually format for telemetryM
        telemetryM.addData(key, formattedValue);
    }

}