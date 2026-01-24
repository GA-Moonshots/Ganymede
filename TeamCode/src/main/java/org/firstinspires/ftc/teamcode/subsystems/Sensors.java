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


public class Sensors extends SubsystemBase {
    private Telemetry telemetry;
    private TelemetryManager telemetryM;
    private Ganymede robot;
    private Limelight3A limelight;

    public RevColorSensorV3 colorSensor;

    // Color detection thresholds
    private static final float GREEN_HUE_MIN = 145f;
    private static final float GREEN_HUE_MAX = 160f;
    private static final float MIN_SATURATION = 0.3f;
    private static final float MIN_VALUE = 0.15f;
    private static final float GREEN_RATIO_THRESHOLD = 1.3f;
    private static final float COLOR_SENSOR_GAIN = 2.0f;

    public enum BallColor {
        GREEN,
        PURPLE,
        NONE
    }

    public Sensors(Ganymede robot) {
        this.robot = robot;
        telemetry = robot.telemetry;
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        colorSensor = robot.hardwareMap.get(RevColorSensorV3.class, Constants.COLOR_SENSOR);
        colorSensor.setGain(COLOR_SENSOR_GAIN);

        // ============================================================
        //              LIMELIGHT INITIALIZATION
        // ============================================================
        try {
            limelight = robot.hardwareMap.get(Limelight3A.class, Constants.LIMELIGHT_NAME);
            limelight.setPollRateHz(50);
            limelight.start();
            limelight.pipelineSwitch(0);  // AprilTag pipeline
        } catch (Exception e) {
            addTelemetry("Limelight Error", e.getMessage());
        }
    }

    // ============================================================
    //              MOTIF DETECTION
    // ============================================================

    /**
     * Passively scans for obelisk AprilTags.
     * @param result Current Limelight result from this cycle
     */
    private void scanForMotif(LLResult result) {
        // Don't scan if already detected
        if (!robot.motif.isEmpty()) {
            return;
        }

        if (result == null || !result.isValid()) {
            return;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

        if (fiducials.isEmpty()) {
            return;
        }

        // Search for obelisk tags (IDs 21, 22, 23)
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int tagId = fiducial.getFiducialId();

            switch (tagId) {
                case 21:
                    robot.motif = "GPP";
                    addTelemetry("MOTIF LOCKED", "GPP (Tag 21)");
                    return;
                case 22:
                    robot.motif = "PGP";
                    addTelemetry("MOTIF LOCKED", "PGP (Tag 22)");
                    return;
                case 23:
                    robot.motif = "PPG";
                    addTelemetry("MOTIF LOCKED", "PPG (Tag 23)");
                    return;
            }
        }
    }

    /**
     * Adds AprilTag telemetry for debugging.
     * @param result Current Limelight result from this cycle
     */
    private void addAprilTagTelemetry(LLResult result) {
        if (result == null || !result.isValid()) {
            return;
        }

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();

        if (tags.isEmpty()) {
            addTelemetry("AprilTags", "None");
            return;
        }

        addTelemetry("AprilTags", "%d detected", tags.size());

        // Show detected tag IDs
        StringBuilder tagIds = new StringBuilder();
        for (LLResultTypes.FiducialResult tag : tags) {
            if (tagIds.length() > 0) tagIds.append(", ");
            tagIds.append(tag.getFiducialId());
        }
        addTelemetry("Tag IDs", tagIds.toString());
    }

    @Override
    public void periodic() {
        // Get Limelight result ONCE per cycle
        LLResult currentResult = null;
        if (limelight != null) {
            currentResult = limelight.getLatestResult();
        }

        // PASSIVE MOTIF SCANNING (pass the result)
        scanForMotif(currentResult);

        // Color sensor telemetry
        addColorTelemetry();

        // Limelight telemetry
        addAprilTagTelemetry(currentResult);

        // Motif status
        if (robot.motif.isEmpty()) {
            addTelemetry("Motif", "Scanning...");
        } else {
            addTelemetry("Motif", robot.motif);
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
        NormalizedRGBA sample = colorSensor.getNormalizedColors();
        BallColor detectedColor = detectBallColor();

        addTelemetry("Ball Color", detectedColor.toString());

        if (detectedColor != BallColor.NONE) {
            addTelemetry("Color Alpha", "%.2f", sample.alpha);
        }
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