package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Ganymede;
import org.firstinspires.ftc.teamcode.utils.Constants;
import java.util.List;

/**
 * ╔═══════════════════════════════════════════════════════════════════════════╗
 * ║                         SENSOR SUBSYSTEM                                  ║
 * ║                                                                           ║
 * ║  Manages all sensors: Color detection, Limelight AprilTags, and more.     ║
 * ║  Handles passive MOTIF scanning during autonomous.                        ║
 * ╚═══════════════════════════════════════════════════════════════════════════╝
 */
public class Sensors extends SubsystemBase {
    private Telemetry telemetry;
    private TelemetryManager telemetryM;
    private Ganymede robot;
    private Limelight3A limelight;  // null means unavailable

    public RevColorSensorV3 colorSensor;

    // ============================================================
    //              COLOR DETECTION CONSTANTS
    // ============================================================
    private static final float GREEN_HUE_MIN = 145f;
    private static final float GREEN_HUE_MAX = 160f;
    private static final float MIN_SATURATION = 0.3f;
    private static final float MIN_VALUE = 0.15f;
    private static final float GREEN_RATIO_THRESHOLD = 1.3f;
    private static final float COLOR_SENSOR_GAIN = 2.0f;

    // ============================================================
    //              BALL COLOR ENUM
    // ============================================================
    public enum BallColor {
        GREEN,
        PURPLE,
        NONE
    }

    // ============================================================
    //                        CONSTRUCTOR
    // ============================================================
    public Sensors(Ganymede robot) {
        this.robot = robot;
        telemetry = robot.telemetry;
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        colorSensor = robot.hardwareMap.get(RevColorSensorV3.class, Constants.COLOR_SENSOR);
        colorSensor.setGain(COLOR_SENSOR_GAIN);

        try {
            limelight = robot.hardwareMap.get(Limelight3A.class, Constants.LIMELIGHT_NAME);
            limelight.setPollRateHz(30);
            limelight.start();
            limelight.pipelineSwitch(0);  // AprilTag pipeline
        } catch (Exception e) {
            limelight = null;
            addTelemetry("Limelight", "OFFLINE: " + e.getMessage());
        }
    }

    // ============================================================
    //              MOTIF DETECTION
    // ============================================================
    /**
     * Passively scans for obelisk AprilTags (IDs 21, 22, 23).
     * @param result Current Limelight result from this cycle
     */
    private void scanForMotif(LLResult result) {
        if (!robot.motif.isEmpty() || result == null || !result.isValid()) return;

        for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
            switch (fiducial.getFiducialId()) {
                case 21: robot.motif = "GPP"; return;
                case 22: robot.motif = "PGP"; return;
                case 23: robot.motif = "PPG"; return;
            }
        }
    }

    // ============================================================
    //              PERIODIC UPDATE
    // ============================================================
    @Override
    public void periodic() {
        LLResult result = null;
        addTelemetry("═══ LIMELIGHT ═══", "");
        if (limelight != null) {
            try {
                result = limelight.getLatestResult();
                boolean valid = result != null && result.isValid();
                addTelemetry("LL Connected", String.valueOf(limelight.isConnected()));
                addTelemetry("LL Result", valid
                        ? String.format("valid | %d fiducials | age %dms",
                                result.getFiducialResults().size(),
                                limelight.getTimeSinceLastUpdate())
                        : String.format("NO DATA | age %dms",
                                limelight.getTimeSinceLastUpdate()));
            } catch (Exception e) {
                limelight = null;
                addTelemetry("LL Connected", "LOST");
            }
        } else {
            addTelemetry("LL Connected", "OFFLINE");
        }

        scanForMotif(result);
        addColorTelemetry();

        if (!robot.motif.isEmpty()) {
            addTelemetry("Motif", robot.motif);
        } else {
            addTelemetry("Motif", limelight == null ? "Manual" : "Scanning...");
        }

        // !!! THIS SHOULD BE THE ONLY TELEMETRY UPDATE IN THE WHOLE PROJECT !!
        telemetryM.update(telemetry);
    }

    // ============================================================
    //              TELEMETRY HELPERS
    // ============================================================
    public void addTelemetry(String key, String value){
        telemetryM.addData(key, value);
    }

    public void addTelemetry(String key, String format, Object... args) {
        String formattedValue = String.format(format, args);
        telemetryM.addData(key, formattedValue);
    }

    private void addColorTelemetry() {
        addTelemetry("Ball Color", detectBallColor().toString());
    }

    // ============================================================
    //              COLOR DETECTION METHODS
    // ============================================================
    public boolean isGreen() {
        NormalizedRGBA sample = colorSensor.getNormalizedColors();

        if (sample.alpha < MIN_VALUE) {
            return false;
        }

        float[] hsv = rgbToHsv(sample.red, sample.green, sample.blue);
        float hue = hsv[0];
        float saturation = hsv[1];
        float value = hsv[2];

        boolean hsvGreen = (hue >= GREEN_HUE_MIN && hue <= GREEN_HUE_MAX)
                && saturation >= MIN_SATURATION
                && value >= MIN_VALUE;

        float greenRatio = sample.green / Math.max(sample.red, sample.blue);
        boolean rgbGreen = greenRatio > GREEN_RATIO_THRESHOLD
                && sample.green > sample.red
                && sample.green > sample.blue;

        return hsvGreen || rgbGreen;
    }

    public boolean isPurple() {
        NormalizedRGBA sample = colorSensor.getNormalizedColors();
        return sample.alpha >= MIN_VALUE && !isGreen();
    }

    public BallColor detectBallColor() {
        NormalizedRGBA sample = colorSensor.getNormalizedColors();

        if (sample.alpha < MIN_VALUE) {
            return BallColor.NONE;
        }

        if (isGreen()) {
            return BallColor.GREEN;
        }

        return BallColor.PURPLE;
    }

    private float[] rgbToHsv(float r, float g, float b) {
        float max = Math.max(r, Math.max(g, b));
        float min = Math.min(r, Math.min(g, b));
        float delta = max - min;

        float hue = 0;
        float saturation = (max == 0) ? 0 : delta / max;
        float value = max;

        if (delta != 0) {
            if (max == r) {
                hue = 60 * (((g - b) / delta) % 6);
            } else if (max == g) {
                hue = 60 * (((b - r) / delta) + 2);
            } else {
                hue = 60 * (((r - g) / delta) + 4);
            }

            if (hue < 0) {
                hue += 360;
            }
        }

        return new float[]{hue, saturation, value};
    }
}
