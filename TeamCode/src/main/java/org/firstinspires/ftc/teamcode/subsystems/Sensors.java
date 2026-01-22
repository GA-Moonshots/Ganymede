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
//    private Limelight3A limelight;

    public RevColorSensorV3 colorSensor;

    // Color detection thresholds - tune these based on testing
    private static final float GREEN_HUE_MIN = 145f;   // Degrees
    private static final float GREEN_HUE_MAX = 160f;   // Degrees
    private static final float MIN_SATURATION = 0.3f;  // Minimum color intensity
    private static final float MIN_VALUE = 0.15f;      // Minimum brightness
    private static final float GREEN_RATIO_THRESHOLD = 1.3f;  // Green must be this much higher than R and B

    // Enum for ball color state
    public enum BallColor {
        GREEN,
        PURPLE,
        NONE
    }

    // Color sensor gain (0-8, higher = more sensitive)
    private static final float COLOR_SENSOR_GAIN = 2.0f;

    public Sensors(Ganymede robot) {
        this.robot = robot;
        telemetry = robot.telemetry;
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        colorSensor = robot.hardwareMap.get(RevColorSensorV3.class, Constants.COLOR_SENSOR);

        // Set gain for better color detection
        colorSensor.setGain(COLOR_SENSOR_GAIN);


    }

    /*public Sensors(Ganymede robot) {
        telemetry = robot.telemetry;
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();


//        try {
//            limelight = robot.hardwareMap.get(Limelight3A.class, Constants.LIMELIGHT_NAME);
//            limelight.setPollRateHz(100);
//            limelight.start();
//            limelight.pipelineSwitch(0);
//        } catch (Exception ignored) {
//
//        }
    } */

//    public LLResult getLLResult() {
//        return limelight.getLatestResult();
//    }

//    private void addAprilTagTelemetry() {
//        LLResult result = limelight.getLatestResult();
//
//        if (result == null) {
//            addTelemetry("Limelight", "No result");
//            return;
//        }
//
//        if (!result.isValid()) {
//            addTelemetry("Limelight", "Invalid frame");
//            return;
//        }
//
//        List<LLResultTypes.BarcodeResult> tags = result.getBarcodeResults();
//
//        if (tags.isEmpty()) {
//            addTelemetry("AprilTags", "None detected");
//            return;
//        }
//
//        addTelemetry("AprilTags Detected", "%d", tags.size());
//    }


    @Override
    public void periodic() {
        // Add color sensor telemetry for debugging
        addColorTelemetry();

        //limlight telemetry
//        addAprilTagTelemetry();


        // !!! THIS SHOULD BE THE ONLY TELEMETRY UPDATE IN THE WHOLE PROJECT !!
        telemetryM.update(telemetry);
    }

    public void addTelemetry(String key, String value){
        telemetryM.addData(key, value);
    }

    public void addTelemetry(String key, String format, Object... args) {
        String formattedValue = String.format(format, args);
        telemetryM.addData(key, formattedValue);
    }

    /**
     * Detects if a green ball is present at the color sensor
     * Uses HSV-based detection with RGB fallback
     */
    public boolean isGreen() {
        NormalizedRGBA sample = colorSensor.getNormalizedColors();

        // Check if we have sufficient signal (ball is present)
        if (sample.alpha < MIN_VALUE) {
            return false;
        }

        // Primary method: HSV-based detection
        float[] hsv = rgbToHsv(sample.red, sample.green, sample.blue);
        float hue = hsv[0];
        float saturation = hsv[1];
        float value = hsv[2];

        // Check HSV criteria
        boolean hsvGreen = (hue >= GREEN_HUE_MIN && hue <= GREEN_HUE_MAX)
                && saturation >= MIN_SATURATION
                && value >= MIN_VALUE;

        // Backup method: RGB ratio-based detection
        float greenRatio = sample.green / Math.max(sample.red, sample.blue);
        boolean rgbGreen = greenRatio > GREEN_RATIO_THRESHOLD
                && sample.green > sample.red
                && sample.green > sample.blue;

        // Return true if either method confirms green
        return hsvGreen || rgbGreen;
    }

    /**
     * Detects if a purple ball is present at the color sensor
     * Simple logic: if there's a ball and it's not green, it's purple
     */
    public boolean isPurple() {
        NormalizedRGBA sample = colorSensor.getNormalizedColors();

        // Ball is present and it's not green = must be purple
        return sample.alpha >= MIN_VALUE && !isGreen();
    }

    /**
     * Determines the current ball color at the sensor
     * @return BallColor enum indicating detected color
     */
    public BallColor detectBallColor() {
        NormalizedRGBA sample = colorSensor.getNormalizedColors();

        // Check if ball is present (sufficient brightness)
        if (sample.alpha < MIN_VALUE) {
            return BallColor.NONE;
        }

        // Check for green
        if (isGreen()) {
            return BallColor.GREEN;
        }

        // If there's a ball and it's not green, it must be purple
        return BallColor.PURPLE;
    }

    /**
     * Converts RGB to HSV color space
     * HSV is often more reliable for color detection than raw RGB
     * @return float array [hue (0-360), saturation (0-1), value (0-1)]
     */
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
        }

        if (hue < 0) {
            hue += 360;
        }

        return new float[]{hue, saturation, value};
    }

    /**
     * Adds color sensor data to telemetry for debugging
     */
    private void addColorTelemetry() {
        NormalizedRGBA sample = colorSensor.getNormalizedColors();
        float[] hsv = rgbToHsv(sample.red, sample.green, sample.blue);

        addTelemetry("Color", "R:%.2f G:%.2f B:%.2f A:%.2f",
                sample.red, sample.green, sample.blue, sample.alpha);
        addTelemetry("HSV", "H:%.0f° S:%.2f V:%.2f",
                hsv[0], hsv[1], hsv[2]);
        addTelemetry("Ball Detected", detectBallColor().toString());
    }
}