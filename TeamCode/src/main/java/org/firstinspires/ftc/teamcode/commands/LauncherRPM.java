package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.hardware.CRServo;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Ganymede;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.Constants;

import java.util.concurrent.TimeUnit;

/**
 * RPM-based launcher command.
 * Spins the flywheel up to launch power, waits until RPM reaches the feed
 * threshold, then feeds the ball in short pulses. After each pulse the feeder
 * stops and the command checks for an RPM drop — confirming the ball passed
 * through. This prevents double-fires and avoids burning the full timeout when
 * back-to-back shots are needed in autonomous.
 */
public class LauncherRPM extends CommandBase {
    private Ganymede robot;
    private Launcher launcher;
    private CRServo feeder;
    private Timing.Timer timer;
    private Timing.Timer pulseTimer;

    int runFeeder;
    double launcherSpeed;

    // When > 0, the motor runs in velocity PID mode targeting this exact RPM.
    // When -1, falls back to raw power (setPower) using launcherSpeed.
    double targetRPM = -1;

    private enum FeedState { SPINNING_UP, PULSE_ON, PULSE_OFF, DONE }
    private FeedState feedState;
    private int pulseCount;
    private boolean ballOutputted;

    public LauncherRPM(Ganymede robot, double launcherSpeed) {
        this.robot = robot;
        this.launcher = this.robot.launcher;
        this.launcherSpeed = launcherSpeed;
        this.timer = new Timing.Timer(
                (long)(Constants.LAUNCHER_RPM_TIMEOUT_SECONDS * 1000), TimeUnit.MILLISECONDS);

        addRequirements(launcher);
    }

    public LauncherRPM(Ganymede robot) {
        this.robot = robot;
        this.launcher = this.robot.launcher;
        this.launcherSpeed = Constants.LAUNCHER_DEFAULT_POWER;
        this.timer = new Timing.Timer(
                (long)(Constants.LAUNCHER_RPM_TIMEOUT_SECONDS * 1000), TimeUnit.MILLISECONDS);

        addRequirements(launcher);
    }

    /**
     * Dynamic constructor — automatically calculates the correct flywheel RPM
     * based on how far the robot currently is from the goal.
     *
     * Why RPM instead of power?
     *   Motor power (0.0–1.0) varies with battery voltage, so the same power
     *   setting fires faster or slower depending on battery charge. Using
     *   setVelocity() lets the motor's built-in PID maintain a consistent RPM.
     *
     * @param robot   the main robot object (provides pose + alliance info)
     * @param dynamic pass true to enable distance-based RPM; false = default power
     */
    public LauncherRPM(Ganymede robot, boolean dynamic) {
        this.robot = robot;
        this.launcher = this.robot.launcher;
        if (dynamic) {
            // Ask PedroDrive for the distance — single source of truth, matches telemetry
            double dist = robot.drive.getDistanceToGoal();
            // Look up the target RPM in the calibration table (Constants.LAUNCHER_RPM_TABLE)
            this.targetRPM = Constants.distanceToRPM(dist);
            this.launcherSpeed = -1; // unused in velocity mode
        } else {
            this.launcherSpeed = Constants.LAUNCHER_DEFAULT_POWER;
        }
        this.timer = new Timing.Timer(
                (long)(Constants.LAUNCHER_RPM_TIMEOUT_SECONDS * 1000), TimeUnit.MILLISECONDS);

        addRequirements(launcher);
    }

    @Override
    public void initialize() {
        super.initialize();
        ballOutputted = false;
        feedState = FeedState.SPINNING_UP;
        pulseCount = 0;

        // Determine the correct feeder and power orientation (green feeds negative)
        if (robot.turret.state == Turret.TurretState.LEFT) {
            feeder = robot.launcher.greenFeeder;
            runFeeder = -1; // reversed because of gearing
        } else {
            feeder = robot.launcher.purpleFeeder;
            runFeeder = 1;
        }

        timer.start();
    }

    @Override
    public void execute() {
        // Velocity mode: motor PID holds the exact RPM regardless of battery level.
        // Power mode: raw 0.0–1.0 — simpler but drifts as battery drains.
        if (targetRPM > 0) {
            // RPM → deg/s conversion: 1 RPM = 6 deg/s (since 360°/60s = 6 deg/s per RPM)
            launcher.launcher.setVelocity(targetRPM * 6, AngleUnit.DEGREES);
            robot.sensors.addTelemetry("Target RPM", String.format("%.0f", targetRPM));
        } else {
            launcher.launcher.setPower(launcherSpeed);
        }

        // Conversion from deg/s to rpm
        double currentSpeed = Math.abs(launcher.launcher.getVelocity(AngleUnit.DEGREES)) / 6;
        robot.sensors.addTelemetry("Current RPM: ", String.valueOf(currentSpeed));

        double activeCurrent = launcher.launcher.getCurrent(CurrentUnit.AMPS);
        robot.sensors.addTelemetry("Motor Current (A): ", String.valueOf(activeCurrent));
        robot.sensors.addTelemetry("Feed State", feedState.name());
        robot.sensors.addTelemetry("Pulse Count", String.valueOf(pulseCount));

        // Thresholds scale with target RPM in velocity mode so the spin-up
        // wait and ball-detection drop are proportional to the actual speed target.
        // In power mode we fall back to the fixed constant.
        double feedReadyRPM  = targetRPM > 0 ? targetRPM * 0.9 : Constants.LAUNCHER_FEED_RPM;
        double dropThreshold = targetRPM > 0 ? targetRPM * 0.8 : Constants.LAUNCHER_FEED_RPM * 0.8;

        switch (feedState) {
            case SPINNING_UP:
                if (currentSpeed >= feedReadyRPM) {  // wait until flywheel is up to speed
                    pulseTimer = new Timing.Timer(Constants.LAUNCHER_PULSE_ON_MS, TimeUnit.MILLISECONDS);
                    pulseTimer.start();
                    feedState = FeedState.PULSE_ON;
                }
                break;

            case PULSE_ON:
                feeder.setPower(runFeeder);
                if (currentSpeed <= dropThreshold) {
                    // RPM drop caught mid-pulse — ball is through
                    feeder.setPower(0);
                    feedState = FeedState.DONE;
                } else if (pulseTimer.done()) {
                    // Pulse window elapsed; open observation gap
                    feeder.setPower(0);
                    pulseTimer = new Timing.Timer(Constants.LAUNCHER_PULSE_OFF_MS, TimeUnit.MILLISECONDS);
                    pulseTimer.start();
                    feedState = FeedState.PULSE_OFF;
                }
                break;

            case PULSE_OFF:
                feeder.setPower(0);
                if (currentSpeed <= dropThreshold) {
                    // RPM drop caught in the observation gap — ball confirmed through
                    feedState = FeedState.DONE;
                } else if (pulseTimer.done()) {
                    // RPM recovered — ball didn't go through this pulse, try again
                    pulseCount++;
                    if (pulseCount >= Constants.LAUNCHER_MAX_PULSES) {
                        feedState = FeedState.DONE; // safety: gave up after max attempts
                    } else {
                        pulseTimer = new Timing.Timer(Constants.LAUNCHER_PULSE_ON_MS, TimeUnit.MILLISECONDS);
                        pulseTimer.start();
                        feedState = FeedState.PULSE_ON;
                    }
                }
                break;

            case DONE:
                feeder.setPower(0);
                ballOutputted = true;
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return ballOutputted || timer.done();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        launcher.stopAll();
    }
}