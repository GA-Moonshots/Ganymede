package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.Ganymede;
import org.firstinspires.ftc.teamcode.utils.Constants;

/**
 * ╔═══════════════════════════════════════════════════════════════════════════╗
 * ║                      DRIVE TO PARK COMMAND                                ║
 * ║                                                                           ║
 * ║  Alliance-aware parking command that drives to the correct observation    ║
 * ║  zone based on robot.isRed.                                               ║
 * ║                                                                           ║
 * ║  Parking Positions:                                                       ║
 * ║    • RED alliance:  (40, 32)                                              ║
 * ║    • BLUE alliance: (105, 32)                                             ║
 * ║                                                                           ║
 * ║  Typical timeout: 5-10 seconds depending on starting position             ║
 * ╚═══════════════════════════════════════════════════════════════════════════╝
 */
public class DriveToPark extends DriveAbstract {

    // ============================================================
    //                     PARKING COORDINATES
    // ============================================================

    /** Red alliance observation zone parking position */
    private static final double RED_PARK_X = 40.0;
    private static final double RED_PARK_Y = 32.0;

    /** Blue alliance observation zone parking position */
    private static final double BLUE_PARK_X = 105.0;
    private static final double BLUE_PARK_Y = 32.0;

    // ============================================================
    //                     COMMAND STATE
    // ============================================================

    private Pose targetPose;
    private boolean finished = false;

    // ============================================================
    //                     CONSTRUCTOR
    // ============================================================

    /**
     * Creates a command to drive to the alliance-appropriate parking spot.
     *
     * The command automatically selects the correct parking position based
     * on robot.isRed, so you don't need to create separate commands for
     * red and blue alliances.
     *
     * @param robot Main robot object (contains alliance color)
     * @param timeoutSeconds Safety timeout in seconds (typically 5-10s)
     */
    public DriveToPark(Ganymede robot, double timeoutSeconds) {
        super(robot, timeoutSeconds);
    }

    // ============================================================
    //                     COMMAND LIFECYCLE
    // ============================================================

    @Override
    public void initialize() {
        timer.start();

        Pose currentPose = follower.getPose();

        // Select parking coordinates based on alliance color
        double parkX = robot.isRed ? RED_PARK_X : BLUE_PARK_X;
        double parkY = robot.isRed ? RED_PARK_Y : BLUE_PARK_Y;

        // Create target pose (maintain current heading to avoid unnecessary rotation)
        targetPose = new Pose(parkX, parkY, 0);

        // Create and follow straight-line path to parking spot
        Path path = new Path(new BezierLine(currentPose, targetPose));
        follower.followPath(path);
    }

    @Override
    public void execute() {
        super.execute();

        // Check if we've reached the parking spot within tolerance
        if (follower.atPose(targetPose, Constants.POSE_TOLERANCE, Constants.POSE_TOLERANCE)) {
            finished = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        standardCleanup();

    }

    @Override
    public boolean isFinished() {
        return finished || timer.done();
    }
}