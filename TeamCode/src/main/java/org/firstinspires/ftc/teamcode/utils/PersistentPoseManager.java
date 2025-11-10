package org.firstinspires.ftc.teamcode.utils;

import android.os.Environment;

import com.pedropathing.geometry.Pose;

import java.io.*;

/**
 * Handles persistent storage of robot pose between OpMode runs.
 * Saves pose to file at end of autonomous, loads at start of teleop.
 */
public class PersistentPoseManager {
    private static final String POSE_FILE_PATH = Environment.getExternalStorageDirectory().getPath()+"/FIRST/pose.txt";

    /**
     * Saves the robot's pose to a file.
     *
     * @param pose The robot's current pose (x, y, heading in radians).
     */
    public static void savePose(Pose pose) {
        try (BufferedWriter writer = new BufferedWriter(new FileWriter(POSE_FILE_PATH))) {
            // Write pose data to file as a comma-separated string
            writer.write(pose.getX() + "," + pose.getY() + "," + pose.getHeading());
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Loads the robot's pose from a file.
     *
     * @return The saved pose, or (105, 32, 0) if the file does not exist or an error occurs.
     */
    public static Pose loadPose() {
        try (BufferedReader reader = new BufferedReader(new FileReader(POSE_FILE_PATH))) {
            // Read pose data from the file
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
        // Default pose if loading fails - center of red observation zone
        return new Pose(105, 32, 0);
    }
}