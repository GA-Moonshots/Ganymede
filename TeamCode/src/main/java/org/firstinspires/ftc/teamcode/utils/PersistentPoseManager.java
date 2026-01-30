package org.firstinspires.ftc.teamcode.utils;

import android.os.Environment;

import com.pedropathing.geometry.Pose;

import java.io.*;

/**
 * ═══════════════════════════════════════════════════════════════════════════
 *                        PERSISTENT POSE MANAGER
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * Handles the handoff between autonomous and teleop.
 *
 * At the end of auto, we save:
 *   - Final robot pose (x, y, heading)
 *   - Alliance color (red/blue)
 *
 * At the start of teleop, we load these separately to:
 *   - Initialize Pedro's localization at the correct spot
 *   - Set up field-centric drive for the right alliance
 *
 * ═══════════════════════════════════════════════════════════════════════════
 */
public class PersistentPoseManager {
    private static final String POSE_FILE_PATH = Environment.getExternalStorageDirectory().getPath() + "/FIRST/pose.txt";

    // Defaults if file doesn't exist or can't be read
    private static final Pose DEFAULT_POSE = new Pose(105, 32, 0); // Center of red observation zone
    private static final boolean DEFAULT_IS_RED = true;

    /**
     * Saves the robot's final pose and alliance color at end of autonomous.
     *
     * File format: x,y,heading,isRed
     * Example: 105.0,32.0,1.5708,false
     *
     * @param pose The robot's final pose (x, y, heading in radians)
     * @param isRed Whether we're on red alliance (true) or blue (false)
     */
    public static void savePose(Pose pose, boolean isRed) {
        try (BufferedWriter writer = new BufferedWriter(new FileWriter(POSE_FILE_PATH))) {
            writer.write(pose.getX() + "," + pose.getY() + "," + pose.getHeading() + "," + isRed);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Loads the robot's pose from file.
     * Call this at the start of teleop to initialize Pedro's localization.
     *
     * @return The saved pose, or default (105, 32, 0) if file doesn't exist
     */
    public static Pose loadPose() {
        try (BufferedReader reader = new BufferedReader(new FileReader(POSE_FILE_PATH))) {
            String line = reader.readLine();
            if (line != null) {
                String[] data = line.split(",");
                double x = Double.parseDouble(data[0]);
                double y = Double.parseDouble(data[1]);
                double heading = Double.parseDouble(data[2]);
                return new Pose(x, y, heading);
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
        return DEFAULT_POSE;
    }

    /**
     * Loads the alliance color from file.
     * Call this at the start of teleop to configure field-centric drive direction.
     *
     * @return true if red alliance, false if blue, or default (true) if file doesn't exist
     */
    public static boolean loadIsRed() {
        try (BufferedReader reader = new BufferedReader(new FileReader(POSE_FILE_PATH))) {
            String line = reader.readLine();
            if (line != null) {
                String[] data = line.split(",");
                // Check if alliance color was saved (handles old file format gracefully)
                if (data.length >= 4) {
                    return Boolean.parseBoolean(data[3]);
                }
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
        return DEFAULT_IS_RED;
    }
}