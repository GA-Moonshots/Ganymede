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
    private Limelight3A limelight;
    private boolean limelightInitialized = false;
    private boolean limelightReady = false;
    private int warmupCyclesRemaining = 0;

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
    //              LIMELIGHT WARMUP SETTINGS
    // ============================================================
    /**
     * Reduced warmup period - cable was the real issue!
     * ~200ms at 50Hz periodic rate
     */
    private static final int WARMUP_CYCLES = 10;

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

        initializeLimelight();
    }

    // ============================================================
    //              LIMELIGHT INITIALIZATION
    // ============================================================
    /**
     * Initializes Limelight with minimal blocking.
     * Actual readiness is determined over time in periodic().
     */
    private void initializeLimelight() {
        try {
            limelight = robot.hardwareMap.get(Limelight3A.class, Constants.LIMELIGHT_NAME);

            limelight.setPollRateHz(30);
            limelight.start();
            limelight.pipelineSwitch(0);  // AprilTag pipeline

            limelightInitialized = true;
            warmupCyclesRemaining = WARMUP_CYCLES;

            addTelemetry("Limelight", "Initializing...");

        } catch (Exception e) {
            limelightInitialized = false;
            limelightReady = false;
            addTelemetry("Limelight Error", e.getMessage());
            addTelemetry("Limelight", "⚠ Disabled - using manual motif");
        }
    }

    // ============================================================
    //              MOTIF DETECTION
    // ============================================================
    /**
     * Passively scans for obelisk AprilTags (IDs 21, 22, 23).
     * Only runs after brief Limelight warmup period.
     * @param result Current Limelight result from this cycle
     */
    private void scanForMotif(LLResult result) {
        // Don't scan if already detected
        if (!robot.motif.isEmpty()) {
            return;
        }

        // Don't scan if Limelight not ready
        if (!limelightReady) {
            return;
        }

        if (result == null || !result.isValid()) {
            addTelemetry("Scan", "Invalid result");
            return;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

        if (fiducials.isEmpty()) {
            addTelemetry("Scan", "No tags visible");
            return;
        }

        addTelemetry("Scan", "Checking %d tag(s)", fiducials.size());

        // Search for obelisk tags (IDs 21, 22, 23)
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int tagId = fiducial.getFiducialId();

            addTelemetry("Scan", "Found Tag %d", tagId);

            switch (tagId) {
                case 21:
                    robot.motif = "GPP";
                    addTelemetry("MOTIF LOCKED", "✓ GPP (Tag 21)");
                    return;
                case 22:
                    robot.motif = "PGP";
                    addTelemetry("MOTIF LOCKED", "✓ PGP (Tag 22)");
                    return;
                case 23:
                    robot.motif = "PPG";
                    addTelemetry("MOTIF LOCKED", "✓ PPG (Tag 23)");
                    return;
            }
        }
    }

    /**
     * Adds AprilTag telemetry for debugging.
     * Only runs after Limelight is ready.
     * @param result Current Limelight result from this cycle
     */
    private void addAprilTagTelemetry(LLResult result) {
        if (!limelightReady) {
            return;
        }

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

    // ============================================================
    //              PERIODIC UPDATE
    // ============================================================
    @Override
    public void periodic() {
        // ============================================================
        //              LIMELIGHT WARMUP MANAGEMENT
        // ============================================================
        if (limelightInitialized && limelight != null && !limelightReady) {
            LLResult r = limelight.getLatestResult();

            // "Ready" means: we are receiving updates recently AND the result is valid
            if (limelight.isConnected() && r != null && r.isValid()) {
                limelightReady = true;
                addTelemetry("Limelight", "✓ Ready (valid results)");
            } else {
                addTelemetry("Limelight", "Warming... conn=%s valid=%s age=%dms",
                        limelight.isConnected(),
                        (r != null && r.isValid()),
                        limelight.getTimeSinceLastUpdate());
            }
        }


        // ============================================================
        //              LIMELIGHT DATA RETRIEVAL
        // ============================================================
        LLResult currentResult = null;
        if (limelightInitialized && limelight != null) {
            try {
                currentResult = limelight.getLatestResult();
            } catch (Exception e) {
                limelightInitialized = false;
                limelightReady = false;
                addTelemetry("Limelight", "⚠ Lost connection");
            }
        }

        // ============================================================
        //              SENSOR DATA PROCESSING
        // ============================================================

        // PASSIVE MOTIF SCANNING (only after warmup)
        scanForMotif(currentResult);

        // Color sensor telemetry
        addColorTelemetry();

        // Limelight telemetry (only if ready)
        if (limelightReady) {
            addAprilTagTelemetry(currentResult);
        }

        // Motif status
        if (robot.motif.isEmpty()) {
            if (!limelightInitialized) {
                addTelemetry("Motif", "Manual - LL offline");
            } else if (!limelightReady) {
                addTelemetry("Motif", "Warming up...");
            } else {
                addTelemetry("Motif", "Scanning...");
            }
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