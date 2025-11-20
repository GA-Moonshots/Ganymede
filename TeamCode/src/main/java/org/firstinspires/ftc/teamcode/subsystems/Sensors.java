package org.firstinspires.ftc.teamcode.subsystems;

import android.telecom.TelecomManager;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.TouchSensor;
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
    private Ganymede robot;

    public NormalizedColorSensor colorSensor;

    // Color detection thresholds - tune these based on testing
    private static final float GREEN_HUE_MIN = 80f;   // Degrees
    private static final float GREEN_HUE_MAX = 160f;  // Degrees
    private static final float PURPLE_HUE_MIN = 260f; // Degrees
    private static final float PURPLE_HUE_MAX = 310f; // Degrees
    private static final float MIN_SATURATION = 0.3f; // Minimum color intensity
    private static final float MIN_VALUE = 0.15f;     // Minimum brightness

    // Alternative RGB-based thresholds (backup method)
    private static final float GREEN_RATIO_THRESHOLD = 1.3f;  // Green must be this much higher than R and B
    private static final float PURPLE_RB_RATIO = 0.8f;        // Red/Blue ratio for purple (close to 1)
    private static final float PURPLE_GREEN_MAX = 0.4f;       // Purple has low green

    // Enum for ball color state
    public enum BallColor {
        GREEN,
        PURPLE,
        UNKNOWN,
        NONE
    }

    // Color sensor gain (0-8, higher = more sensitive)
    private static final float COLOR_SENSOR_GAIN = 2.0f;

    public Sensors(Ganymede robot) {
        this.robot = robot;
        telemetry = robot.telemetry;
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        colorSensor = robot.hardwareMap.get(NormalizedColorSensor.class, Constants.COLOR_SENSOR);

        // Set gain for better color detection
        colorSensor.setGain(COLOR_SENSOR_GAIN);

    }


    @Override
    public void periodic() {
        // Add color sensor telemetry for debugging
        addColorTelemetry();

        // !!! THIS SHOULD BE THE ONLY TELEMETRY UPDATE IN THE WHOLE PROJECT !!
        telemetryM.update(telemetry);
    }

    public void addTelemetry(String key, String value){
        telemetryM.addData(key, value);
    }

    public void addTelemetry(String key, String format, Object... args) {
        //telemetry.addData(key, format, args);
        String formattedValue = String.format(format, args); // Manually format for telemetryM
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
     * Purple typically has high red and blue, low green
     */
    public boolean isPurple() {
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

        // Check HSV criteria for purple
        boolean hsvPurple = (hue >= PURPLE_HUE_MIN && hue <= PURPLE_HUE_MAX)
                && saturation >= MIN_SATURATION
                && value >= MIN_VALUE;

        // Backup method: RGB ratio-based detection
        // Purple has similar red and blue, low green
        float rbRatio = sample.red / Math.max(sample.blue, 0.01f);
        float greenLevel = sample.green / Math.max(sample.red, sample.blue);

        boolean rgbPurple = Math.abs(rbRatio - 1.0f) < (1.0f - PURPLE_RB_RATIO)
                && greenLevel < PURPLE_GREEN_MAX
                && sample.red > 0.1f && sample.blue > 0.1f;

        // Return true if either method confirms purple
        return hsvPurple || rgbPurple;
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

        // Check for green first
        if (isGreen()) {
            return BallColor.GREEN;
        }

        // Check for purple
        if (isPurple()) {
            return BallColor.PURPLE;
        }

        // Ball present but color not recognized
        return BallColor.UNKNOWN;
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