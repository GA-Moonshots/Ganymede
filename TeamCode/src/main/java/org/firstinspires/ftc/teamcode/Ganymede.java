package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.Robot;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.Drive;
import org.firstinspires.ftc.teamcode.commands.DriveToPark;
import org.firstinspires.ftc.teamcode.commands.DriveToPose;
import org.firstinspires.ftc.teamcode.commands.DriveRotate;
import org.firstinspires.ftc.teamcode.commands.DriveFwdByDist;
import org.firstinspires.ftc.teamcode.commands.IntakeByDirection;
import org.firstinspires.ftc.teamcode.commands.LauncherIdle;
import org.firstinspires.ftc.teamcode.utils.old.LauncherLaunch;
import org.firstinspires.ftc.teamcode.commands.LauncherRPM;
import org.firstinspires.ftc.teamcode.commands.LauncherRawPower;
import org.firstinspires.ftc.teamcode.commands.TurretRotate;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.PedroDrive;
import org.firstinspires.ftc.teamcode.subsystems.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utils.PersistentPoseManager;

/**
 * Main robot class for the 2025 FTC season using SolversLib and Pedro Pathing
 */
public class Ganymede extends Robot {
    // Core OpMode references
    public LinearOpMode opMode;
    public GamepadEx player1;
    public GamepadEx player2;

    // Match configuration
    public boolean isRed;
    public boolean isNearGoal;
    public String motif = "";

    // SUBSYSTEMS
    public PedroDrive drive;
    public Sensors sensors;
    public Intake intake;
    public Launcher launcher;
    public Turret turret;

    // Convenience references
    public Telemetry telemetry;
    public HardwareMap hardwareMap;

    // Pedro Pathing specific
    public Pose startPose;
    private final double RED_AUTO_TURN = 100;
    private final double BLUE_AUTO_TURN = 115;

    /**
     * TELEOP MODE [--Constructor--]
     */
    public Ganymede(LinearOpMode opMode) {
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;
        this.hardwareMap = opMode.hardwareMap;

        // Initialize gamepads
        player1 = new GamepadEx(opMode.gamepad1);
        player2 = new GamepadEx(opMode.gamepad2);

        // Load starting pose from file -
        startPose = PersistentPoseManager.loadPose();
        isRed = PersistentPoseManager.loadIsRed();

        initTeleOp();
    }

    /**
     * AUTONOMOUS MODE [--Constructor--]
     */
    public Ganymede(LinearOpMode opMode, boolean isRed, boolean isNearGoal) {
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;
        this.hardwareMap = opMode.hardwareMap;
        this.isRed = isRed;
        this.isNearGoal = isNearGoal;

        // Initialize gamepads (may not be used in auto but keeps consistency)
        player1 = new GamepadEx(opMode.gamepad1);
        player2 = new GamepadEx(opMode.gamepad2);

        // Set starting pose based on alliance and side
        if (isRed) {
            if (isNearGoal) {
                // Red near goal - facing OBELISK (90°) for MOTIF detection
                startPose = new Pose(115, 125, Math.toRadians(90));
            } else {
                // Red far goal - just shoot, no motif detection
                startPose = new Pose(80, 12, Math.toRadians(64));
            }
        } else {
            if (isNearGoal) {
                // Blue near goal - facing OBELISK (90°) for MOTIF detection
                startPose = new Pose(15, 122, Math.toRadians(90));
            } else {
                // Blue far goal - just shoot, no motif detection
                startPose = new Pose(50, 12, Math.toRadians(109));
            }
        }
        initAuto();
    }

    /**
     * TELEOP MODE [--Initialize--]
     */
    public void initTeleOp() {
        // Initialize drive subsystem with Pedro Pathing
        drive = new PedroDrive(this, startPose);
        drive.follower.startTeleopDrive();

        // Initialize sensor package (mostly unchanged from your original)
        launcher = new Launcher(this);
        intake = new Intake(this);
        turret = new Turret(this);
        sensors = new Sensors(this);

        // Register subsystems with the command scheduler
        register(drive, sensors, intake, launcher,turret);

        // Set default commands
        drive.setDefaultCommand(new Drive(this));
        launcher.setDefaultCommand(new LauncherIdle(this));

        /*
                .__                                      ____
        ______  |  |  _____   ___.__.  ____ _______     /_   |
        \____ \ |  |  \__  \ <   |  |_/ __ \\_  __ \     |   |
        |  |_> >|  |__ / __ \_\___  |\  ___/ |  | \/     |   |
        |   __/ |____/(____  // ____| \___  >|__|        |___|
        |__|               \/ \/          \/

        */

        // A Button - Reset heading
        new GamepadButton(player1, GamepadKeys.Button.A)
                .whenPressed(() -> drive.resetHeading());

        // B Button - Toggle field-centric mode
        new GamepadButton(player1, GamepadKeys.Button.B)
                .whenPressed(() -> drive.toggleFieldCentric());

        // X Button - Drive to the parking zone for your team
        new GamepadButton(player1, GamepadKeys.Button.X)
                .whenPressed(() -> new DriveToPark(this, 30).schedule());

        // Y Button
        new GamepadButton(player1, GamepadKeys.Button.Y);

        // BACK button
        new GamepadButton(player1, GamepadKeys.Button.BACK)
                .whenPressed(new InstantCommand(() -> {
                    Pose redBaseZone = new Pose(40, 32, 0); // Center of red BASE, facing blue
                    drive.setPose(redBaseZone);
                    sensors.addTelemetry("✓ Relocalized", "Red BASE ZONE");
                }));

        // D-PAD UP - break follower
        new GamepadButton(player1, GamepadKeys.Button.DPAD_UP)
                .whenPressed(() -> {this.drive.follower.breakFollowing();
                    this.drive.follower.startTeleopDrive();});

        new GamepadButton(player1, GamepadKeys.Button.DPAD_LEFT);

        new GamepadButton(player1, GamepadKeys.Button.DPAD_RIGHT);

        /*
                _                                    __
               (_ )                                /'__`\
         _ _    | |    _ _  _   _    __   _ __    (_)  ) )
        ( '_`\  | |  /'_` )( ) ( ) /'__`\( '__)      /' /
        | (_) ) | | ( (_| || (_) |(  ___/| |       /' /( )
        | ,__/'(___)`\__,_)`\__, |`\____)(_)      (_____/'
        | |                ( )_| |
        (_)                `\___/'
        */


        // Button A and B -- Turret
        new GamepadButton(player2, GamepadKeys.Button.A)
                .whenPressed(new TurretRotate(this, Turret.TurretState.LEFT ));

        new GamepadButton(player2, GamepadKeys.Button.B)
                .whenPressed(new TurretRotate(this, Turret.TurretState.FRONT ));

        // Button Y -- Launcher RPM-based launch
        new GamepadButton(player2, GamepadKeys.Button.Y)
                .whenPressed(new LauncherRPM(this));

        // D-PAD intake servos
        new GamepadButton(player2, GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(() -> {intake.stopperG.setPosition(0.5);}));

        new GamepadButton(player2, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(() -> {intake.stopperG.setPosition(0);}));

        new GamepadButton(player2, GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new InstantCommand(() -> {intake.stopperP.setPosition(0.5);}));

        new GamepadButton(player2, GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new InstantCommand(() -> {intake.stopperP.setPosition(0);}));

        // RIGHT TRIGGER -- RAW POWER THE LAUNCHER
        Trigger rightTriggerP2 = new Trigger(() -> player2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5);
        rightTriggerP2.whileActiveContinuous(new LauncherRawPower(this));

        // RIGHT BUMPER - Launcher launch
        new GamepadButton(player2, GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(() -> new LauncherRPM(this).schedule());

        // LEFT BUMPER - reverse intake
        new GamepadButton(player2, GamepadKeys.Button.LEFT_BUMPER)
                .whenHeld(new IntakeByDirection(this, false));

        // LEFT TRIGGER -- INTAKE
        Trigger leftTriggerP2 = new Trigger(() -> player2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5);
        leftTriggerP2.whileActiveContinuous(new IntakeByDirection(this, true));
    }
    /**
     * AUTONOMOUS MODE [--Initialize--]
     *
     * NEAR GOAL: Robot faces OBELISK (90°) for MOTIF detection, waits for scan, then shoots based on pattern.
     * FAR GOAL: Just launch a few balls - too far for reliable pattern scoring.
     */
    public void initAuto() {
        drive = new PedroDrive(this, startPose);
        sensors = new Sensors(this);
        launcher = new Launcher(this);
        turret = new Turret(this);
        intake = new Intake(this);
        register(drive, sensors, launcher, turret, intake);

        drive.update(); // make sure we update our localization before we start moving

        // ════════════════════════════════════════════════════════
        //                BLUE ALLIANCE - NEAR GOAL
        // ════════════════════════════════════════════════════════
        if (!isRed && isNearGoal) {
            // Common setup: Drive to position facing obelisk, then wait for scan
            SequentialCommandGroup setup = new SequentialCommandGroup(
                    new DriveToPose(this, new Pose(50, 90, Math.toRadians(90)), 5),
                    // old values    new DriveToPose(this, new Pose(60, 88, Math.toRadians(90)), 5),
                    new WaitCommand(500)  // Give scanner 0.5s to detect
            );

            // After setup, check robot.motif and execute appropriate shooting sequence
            // TODO: Check if BlUE_AUTO and RED_Auto accounts for the rotation needed to come back from our new angle
            setup.andThen(new InstantCommand(() -> {
                // MOTIF: GREEN - PURPLE - PURPLE
                if (motif.equals("GPP")) {
                    new SequentialCommandGroup(
                            new DriveRotate(this, 65, 2), // putting this here to aim first
                            // FIRE GREEN FIRST (turret LEFT + robot rotates clockwise)
                            new ParallelCommandGroup(
                                    new TurretRotate(this, Turret.TurretState.LEFT),
                                    new DriveRotate(this, BLUE_AUTO_TURN, 3)
                            ),
                            new LauncherLaunch(this, 0.65),  // Green
                            // FIRE PURPLE x2 (turret FRONT + robot back to original heading)
                            new ParallelCommandGroup(
                                    new TurretRotate(this, Turret.TurretState.FRONT),
                                    new DriveRotate(this, -BLUE_AUTO_TURN, 3)
                            ),
                            new LauncherLaunch(this, 0.63),  // Purple 1
                            new LauncherLaunch(this, 0.6),  // Purple 2
                            new DriveToPose(this,
                                    new Pose(21.5, 81.5, Math.toRadians(174.4)), 3)
                    ).schedule();
                }
                // MOTIF: PURPLE - GREEN - PURPLE
                else if (motif.equals("PGP")) {
                    new SequentialCommandGroup(
                            // FIRE PURPLE FIRST (turret FRONT)
                            new TurretRotate(this, Turret.TurretState.FRONT),
                            new DriveRotate(this, 65, 2),
                            new LauncherLaunch(this, 0.65),  // Purple 1
                            // FIRE GREEN (turret LEFT + robot rotates clockwise)
                            new ParallelCommandGroup(
                                    new TurretRotate(this, Turret.TurretState.LEFT),
                                    new DriveRotate(this, BLUE_AUTO_TURN, 3) //TODO: Check if this value is ok
                            ),
                            new LauncherLaunch(this, 0.63),  // Green
                            // FIRE PURPLE (turret FRONT + robot back to original heading)
                            new ParallelCommandGroup(
                                    new TurretRotate(this, Turret.TurretState.FRONT),
                                    new DriveRotate(this, -BLUE_AUTO_TURN, 3)
                            ),
                            new LauncherLaunch(this, 0.6),  // Purple 2
                            new DriveToPose(this,
                                    new Pose(21.5, 81.5, Math.toRadians(174.4)), 3)
                    ).schedule();
                }
                // MOTIF: PURPLE - PURPLE - GREEN (DEFAULT)
                else {  // PPG or no detection
                    new SequentialCommandGroup(
                            // FIRE PURPLE x2 FIRST (turret FRONT)
                            new TurretRotate(this, Turret.TurretState.FRONT),
                            new DriveRotate(this, 65, 2),
                            new LauncherLaunch(this, 0.65),  // Purple 1
                            new LauncherLaunch(this, 0.63),  // Purple 2
                            // FIRE GREEN (turret LEFT + robot rotates clockwise)
                            new ParallelCommandGroup(
                                    new TurretRotate(this, Turret.TurretState.LEFT),
                                    new DriveRotate(this, 110, 3)
                            ),
                            new LauncherLaunch(this, 0.6),  // Green
                            new ParallelCommandGroup(
                                    new DriveToPose(this,
                                            new Pose(21, 81.5, Math.toRadians(174.4)), 3),
                                    new TurretRotate(this, Turret.TurretState.FRONT)
                            )
                    ).schedule();
                }
            })).schedule();
        }

        // ════════════════════════════════════════════════════════
        //                RED ALLIANCE - NEAR GOAL
        // ════════════════════════════════════════════════════════
        else if (isRed && isNearGoal) {
            // Common setup: Drive to position facing obelisk, then wait for scan
            SequentialCommandGroup setup = new SequentialCommandGroup(
                    new DriveToPose(this, new Pose(89, 110, Math.toRadians(90)), 5),
                    // used to be 77 and 88
                    new WaitCommand(500)  // Give scanner 0.5s to detect
            );

            // After setup, check robot.motif and execute appropriate shooting sequence
            // TODO: Check if the Auto angles here is good me kinda guessing
            setup.andThen(new InstantCommand(() -> {
                // MOTIF: GREEN - PURPLE - PURPLE
                if (motif.equals("GPP")) {
                    new SequentialCommandGroup(
                            // FIRE GREEN FIRST (turret LEFT + robot rotates clockwise)
                            new DriveRotate(this, -87, 2),
                            new ParallelCommandGroup(
                                    new TurretRotate(this, Turret.TurretState.LEFT),
                                    new DriveRotate(this, RED_AUTO_TURN, 3)
                            ),
                            new LauncherLaunch(this, 0.65),  // Green
                            // FIRE PURPLE x2 (turret FRONT + robot back to original heading)
                            new ParallelCommandGroup(
                                    new TurretRotate(this, Turret.TurretState.FRONT),
                                    new DriveRotate(this, -RED_AUTO_TURN, 3)
                            ),
                            new LauncherLaunch(this, 0.63),  // Purple 1
                            new LauncherLaunch(this, 0.6),  // Purple 2
                            new DriveToPose(this,
                                    new Pose(109, 86, Math.toRadians(174.4)), 3)
                    ).schedule();
                }
                // MOTIF: PURPLE - GREEN - PURPLE
                else if (motif.equals("PGP")) {
                    new SequentialCommandGroup(
                            // FIRE PURPLE FIRST (turret FRONT)
                            new TurretRotate(this, Turret.TurretState.FRONT),
                            new DriveRotate(this, -87, 2),
                            new LauncherLaunch(this, 0.65),  // Purple 1
                            // FIRE GREEN (turret LEFT + robot rotates clockwise)
                            new ParallelCommandGroup(
                                    new TurretRotate(this, Turret.TurretState.LEFT),
                                    new DriveRotate(this, RED_AUTO_TURN, 3)
                            ),
                            new LauncherLaunch(this, 0.63),  // Green
                            // FIRE PURPLE (turret FRONT + robot back to original heading)
                            new ParallelCommandGroup(
                                    new TurretRotate(this, Turret.TurretState.FRONT),
                                    new DriveRotate(this, -RED_AUTO_TURN, 3)
                            ),
                            new LauncherLaunch(this, 0.6),  // Purple 2
                            new DriveToPose(this,
                                    new Pose(109, 86, Math.toRadians(174.4)), 3)
                    ).schedule();
                }
                // MOTIF: PURPLE - PURPLE - GREEN (DEFAULT)
                else {  // PPG or no detection
                    new SequentialCommandGroup(
                            // FIRE PURPLE x2 FIRST (turret FRONT)
                            new TurretRotate(this, Turret.TurretState.FRONT),
                            new DriveRotate(this, -87, 2),
                            new LauncherLaunch(this, 0.65),  // Purple 1
                            new LauncherLaunch(this, 0.63),  // Purple 2
                            // FIRE GREEN (turret LEFT + robot rotates clockwise)
                            new ParallelCommandGroup(
                                    new TurretRotate(this, Turret.TurretState.LEFT),
                                    new DriveRotate(this, 120, 3)
                            ),
                            new LauncherLaunch(this, 0.6),  // Green
                            new ParallelCommandGroup(
                                    new DriveToPose(this,
                                            new Pose(109, 90, Math.toRadians(174.4)), 3),
                                    // x used to be 109
                                    new TurretRotate(this, Turret.TurretState.FRONT)
                            )
                    ).schedule();
                }
            })).schedule();
        }

        // ════════════════════════════════════════════════════════
        //            FAR GOAL - JUST LAUNCH (NO MOTIF)
        // ════════════════════════════════════════════════════════
        else {
            // Too far for reliable pattern scoring - just shoot some bal
            new SequentialCommandGroup(
                    new TurretRotate(this, Turret.TurretState.FRONT),
                    new LauncherLaunch(this, .8),  // Launch 1
                    new LauncherLaunch(this, .8),  // Launch 2
                    new DriveFwdByDist(this, 24, 20)  // Move forward
            ).schedule();
        }
    }
}