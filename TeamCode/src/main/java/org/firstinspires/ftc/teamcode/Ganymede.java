package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.Robot;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
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
import org.firstinspires.ftc.teamcode.commands.LauncherLaunch;
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
    public String motif;

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
    public Ganymede(LinearOpMode opMode, boolean isRed, boolean isNearGoal, String motif) {
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;
        this.hardwareMap = opMode.hardwareMap;
        this.isRed = isRed;
        this.isNearGoal = isNearGoal;
        this.motif = motif;

        // Initialize gamepads (may not be used in auto but keeps consistency)
        player1 = new GamepadEx(opMode.gamepad1);
        player2 = new GamepadEx(opMode.gamepad2);

        // Set starting pose based on alliance and side
        if (isRed) {
            if (isNearGoal) {
                // Red Left starting position
                startPose = new Pose(115, 125, Math.toRadians(45));
            } else {
                // Red small triangle starting position
                startPose = new Pose(80, 12, Math.toRadians(90));
            }
        } else {
            if (isNearGoal) {
                // Blue Left starting position
                startPose = new Pose(15, 122, Math.toRadians(135.3));
            } else {
                // Blue small triangle starting position
                startPose = new Pose(50, 12, Math.toRadians(90));
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

        // Button Y -- Launcher LAUNCH
        new GamepadButton(player2, GamepadKeys.Button.Y)
                .whenPressed(new LauncherLaunch(this));

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
                .whenPressed(() -> new LauncherLaunch(this).schedule());

        // LEFT BUMPER - reverse intake
        new GamepadButton(player2, GamepadKeys.Button.LEFT_BUMPER)
                .whenHeld(new IntakeByDirection(this, false));

        // LEFT TRIGGER -- INTAKE
        Trigger leftTriggerP2 = new Trigger(() -> player2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5);
        leftTriggerP2.whileActiveContinuous(new IntakeByDirection(this, true));
    }
    /**
     * AUTONOMOUS MODE [--Initialize--]
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
        //                    BLUE ALLIANCE SHOOT
        // ════════════════════════════════════════════════════════
        if(!isRed && isNearGoal) {

            // DUMB SHOOT - NO MOTIF (Just shoot 2 purples)
            if(motif == null || motif.isEmpty()) {
                new SequentialCommandGroup(
                        new DriveToPose(this,
                                new Pose(60, 88, this.drive.follower.getHeading()), 5),
                        new TurretRotate(this, Turret.TurretState.FRONT),
                        new LauncherLaunch(this),  // Purple 1
                        new LauncherLaunch(this),  // Purple 2
                        new DriveToPose(this,
                                new Pose(60, 55, this.drive.follower.getHeading()), 5)
                ).schedule();
            }

            // MOTIF: GREEN - PURPLE - PURPLE
            else if(motif.equals("GPP")) {
                new SequentialCommandGroup(
                        new DriveToPose(this,
                                new Pose(60, 88, this.drive.follower.getHeading()), 5),
                        // FIRE GREEN FIRST (turret LEFT + robot rotates clockwise)
                        new ParallelCommandGroup(
                                new TurretRotate(this, Turret.TurretState.LEFT),
                                new DriveRotate(this, BLUE_AUTO_TURN, 5)  // Clockwise to compensate
                        ),
                        new LauncherLaunch(this, 0.8),  // Green
                        // FIRE PURPLE x2 (turret FRONT + robot back to original heading)
                        new ParallelCommandGroup(
                                new TurretRotate(this, Turret.TurretState.FRONT),
                                new DriveRotate(this, -BLUE_AUTO_TURN, 5)  // Counter-clockwise back
                        ),
                        new LauncherLaunch(this, 0.8),  // Purple 1
                        new LauncherLaunch(this, 0.8 ),  // Purple 2
                        new DriveToPose(this,
                                new Pose(31.5, 81.5, 174.4), 5)
                ).schedule();
            }

            // MOTIF: PURPLE - GREEN - PURPLE
            else if(motif.equals("PGP")) {
                new SequentialCommandGroup(
                        new DriveToPose(this,
                                new Pose(60, 88, this.drive.follower.getHeading()), 5),
                        // FIRE PURPLE FIRST (turret FRONT)
                        new TurretRotate(this, Turret.TurretState.FRONT),
                        new LauncherLaunch(this, 0.8),  // Purple 1
                        // FIRE GREEN (turret LEFT + robot rotates clockwise)
                        new ParallelCommandGroup(
                                new TurretRotate(this, Turret.TurretState.LEFT),
                                new DriveRotate(this, BLUE_AUTO_TURN, 5)  // Clockwise to compensate
                        ),
                        new LauncherLaunch(this, 0.8),  // Green
                        // FIRE PURPLE (turret FRONT + robot back to original heading)
                        new ParallelCommandGroup(
                                new TurretRotate(this, Turret.TurretState.FRONT),
                                new DriveRotate(this, -BLUE_AUTO_TURN, 5)  // Counter-clockwise back
                        ),
                        new LauncherLaunch(this, 0.8),  // Purple 2
                        new DriveToPose(this,
                                new Pose(31.5, 81.5, 174.4), 5)
                ).schedule();
            }

            // MOTIF: PURPLE - PURPLE - GREEN
            else if(motif.equals("PPG")) {
                new SequentialCommandGroup(
                        new DriveToPose(this,
                                new Pose(60, 88, this.drive.follower.getHeading()), 5),
                        // FIRE PURPLE x2 FIRST (turret FRONT)
                        new TurretRotate(this, Turret.TurretState.FRONT),
                        new LauncherLaunch(this, 0.8),  // Purple 1
                        new LauncherLaunch(this, 0.78),  // Purple 2
                        // FIRE GREEN (turret LEFT + robot rotates clockwise)
                        new ParallelCommandGroup(
                                new TurretRotate(this, Turret.TurretState.LEFT),
                                new DriveRotate(this, 110, 5)  // Clockwise to compensate
                        ),
                        new LauncherLaunch(this, 0.78),  // Green
                        new DriveToPose(this,
                                new Pose(31.5, 81.5, 174.4), 5)
                ).schedule();
            }
        }

        // ════════════════════════════════════════════════════════
        //                    RED ALLIANCE SHOOT
        // ════════════════════════════════════════════════════════
        else if(isRed && isNearGoal) {

            // DUMB SHOOT - NO MOTIF (Just shoot 2 purples)
            if(motif == null || motif.isEmpty()) {
                new SequentialCommandGroup(
                        new DriveToPose(this,
                                new Pose(77, 88, this.drive.follower.getHeading()), 5),
                        new LauncherLaunch(this),  // Purple 1
                        new LauncherLaunch(this),  // Purple 2
                        new DriveToPose(this,
                                new Pose(77, 55, this.drive.follower.getHeading()), 5)
                ).schedule();
            }

            // MOTIF: GREEN - PURPLE - PURPLE
            else if(motif.equals("GPP")) {
                new SequentialCommandGroup(
                        new DriveToPose(this,
                                new Pose(77, 88, this.drive.follower.getHeading()), 5),
                        // FIRE GREEN FIRST (turret LEFT + robot rotates clockwise)
                        new ParallelCommandGroup(
                                new TurretRotate(this, Turret.TurretState.LEFT),
                                new DriveRotate(this, RED_AUTO_TURN, 5)  // Clockwise to compensate
                        ),
                        new LauncherLaunch(this, 0.8),  // Green
                        // FIRE PURPLE x2 (turret FRONT + robot back to original heading)
                        new ParallelCommandGroup(
                                new TurretRotate(this, Turret.TurretState.FRONT),
                                new DriveRotate(this, -RED_AUTO_TURN, 5)  // Counter-clockwise back
                        ),
                        new LauncherLaunch(this, 0.85),  // Purple 1
                        new LauncherLaunch(this, 0.83),  // Purple 2
                        new DriveToPose(this,
                                new Pose(109, 86, 174.4), 5)
                ).schedule();
            }

            // MOTIF: PURPLE - GREEN - PURPLE
            else if(motif.equals("PGP")) {
                new SequentialCommandGroup(
                        new DriveToPose(this,
                                new Pose(77, 88, this.drive.follower.getHeading()), 5),
                        // FIRE PURPLE FIRST (turret FRONT)
                        new TurretRotate(this, Turret.TurretState.FRONT),
                        new LauncherLaunch(this, 0.85),  // Purple 1
                        // FIRE GREEN (turret LEFT + robot rotates clockwise)
                        new ParallelCommandGroup(
                                new TurretRotate(this, Turret.TurretState.LEFT),
                                new DriveRotate(this, RED_AUTO_TURN, 5)  // Clockwise to compensate
                        ),
                        new LauncherLaunch(this, 0.8),  // Green
                        // FIRE PURPLE (turret FRONT + robot back to original heading)
                        new ParallelCommandGroup(
                                new TurretRotate(this, Turret.TurretState.FRONT),
                                new DriveRotate(this, -RED_AUTO_TURN, 5)  // Counter-clockwise back
                        ),
                        new LauncherLaunch(this, 0.85),  // Purple 2
                        new DriveToPose(this,
                                new Pose(109, 86, 174.4), 5)
                ).schedule();
            }

            // MOTIF: PURPLE - PURPLE - GREEN
            else if(motif.equals("PPG")) {
                new SequentialCommandGroup(
                        new DriveToPose(this,
                                new Pose(77, 88, this.drive.follower.getHeading()), 5),
                        // FIRE PURPLE x2 FIRST (turret FRONT)
                        new TurretRotate(this, Turret.TurretState.FRONT),
                        new LauncherLaunch(this, 0.85),  // Purple 1
                        new LauncherLaunch(this, 0.83),  // Purple 2
                        // FIRE GREEN (turret LEFT + robot rotates clockwise)
                        new ParallelCommandGroup(
                                new TurretRotate(this, Turret.TurretState.LEFT),
                                new DriveRotate(this, RED_AUTO_TURN, 5)  // Clockwise to compensate
                        ),
                        new LauncherLaunch(this, 0.8),  // Green
                        new DriveToPose(this,
                                new Pose(109, 86, 174.4), 5)
                ).schedule();
            }
        }

        // ════════════════════════════════════════════════════════
        //                    DEFAULT FALLBACK
        // ════════════════════════════════════════════════════════
        else {
            new SequentialCommandGroup(
                    new DriveFwdByDist(this, 24, 20)
            ).schedule();
        }
    }

}