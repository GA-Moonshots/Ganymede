package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.InstantCommand;
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
import org.firstinspires.ftc.teamcode.commands.DriveFwdByDist;
import org.firstinspires.ftc.teamcode.commands.IntakeByDirection;
import org.firstinspires.ftc.teamcode.commands.LauncherLaunch;
import org.firstinspires.ftc.teamcode.commands.LauncherRawPower;
import org.firstinspires.ftc.teamcode.commands.ShootGreen;
import org.firstinspires.ftc.teamcode.commands.ShootPurple;
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
                // Red Left starting position
                startPose = new Pose(115, 125, Math.toRadians(133));
            } else {
                // Red small triangle starting position
                startPose = new Pose(80, 12, Math.toRadians(90));
            }
        } else {
            if (isNearGoal) {
                // Blue Left starting position
                startPose = new Pose(15, 122, Math.toRadians(-131));
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

        // Y Button - Toggle field-centric mode
        new GamepadButton(player1, GamepadKeys.Button.Y)
                .whenPressed(() -> new DriveFwdByDist(this, 24,20).schedule());

        new GamepadButton(player1, GamepadKeys.Button.BACK)
                .whenPressed(new InstantCommand(() -> {
                    Pose redBaseZone = new Pose(40, 32, 0); // Center of red BASE, facing blue
                    drive.setPose(redBaseZone);
                    sensors.addTelemetry("✓ Relocalized", "Red BASE ZONE");
                }));

        new GamepadButton(player1, GamepadKeys.Button.DPAD_UP)
                .whenPressed(() -> {this.drive.follower.breakFollowing();
                    this.drive.follower.startTeleopDrive();});

        new GamepadButton(player1, GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new ShootPurple(this));

        new GamepadButton(player1, GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new ShootGreen(this));

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

        // Button X -- STOPPER
        new GamepadButton(player2, GamepadKeys.Button.X)
                .whenHeld(new InstantCommand (() -> {
                    launcher.greenFeeder.setPower(1);} ));
        new GamepadButton(player2, GamepadKeys.Button.X)
                .whenReleased(new InstantCommand (() -> {
                    launcher.greenFeeder.setPower(0);}   ));

        // D-PAD LEFT -- INTAKE SERVO
        new GamepadButton(player2, GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new InstantCommand (() -> {
                    intake.sorterServo.setPosition(0); } ));

        // D-PAD RIGHT - INTAKE SERVO
        new GamepadButton(player2, GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new InstantCommand (() -> {
                    intake.sorterServo.setPosition(1); } ));

        // RIGHT TRIGGER -- POWER THE LAUNCHER
        Trigger rightTriggerP2 = new Trigger(() -> player2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5);
        rightTriggerP2.whileActiveContinuous(new LauncherRawPower(this));

        // RIGHT BUMPER - Launcher launch
        new GamepadButton(player2, GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(() -> new LauncherLaunch(this).schedule());

        // LEFT BUMPER
        new GamepadButton(player2, GamepadKeys.Button.LEFT_BUMPER);

        // LEFT TRIGGER -- INTAKE
        Trigger leftTriggerP2 = new Trigger(() -> player2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5);
        leftTriggerP2.whileActiveContinuous(new IntakeByDirection(this, true));

        // LEFT BUMPER -- OUTTAKE
        //new GamepadButton(player2, GamepadKeys.Button.LEFT_BUMPER).whileHeld(new IntakeByDirection(this, false));
    }

    /**
     * AUTONOMOUS MODE [--Initialize--]
     */
    public void initAuto() {
        drive = new PedroDrive(this, startPose);
        sensors = new Sensors(this);
        launcher = new Launcher(this);

        // Register subsystems
        register(drive, sensors, launcher);

        drive.update(); // make sure we update our localization before we start moving

        // BLUE SHOOT
        if(!isRed && isNearGoal) {
            new SequentialCommandGroup(
                    new DriveToPose(this,
                            new Pose(60, 88, this.drive.follower.getHeading()), 5),
                    new LauncherLaunch(this),
                    new DriveToPose(this,
                            new Pose(60, 55, this.drive.follower.getHeading()), 5)
            ).schedule();
        }
        // RED SHOOT
        else if(isRed && isNearGoal) {
            new SequentialCommandGroup(
                    new DriveToPose(this,
                            new Pose(77, 88, this.drive.follower.getHeading()), 5),
                    new LauncherLaunch(this),
                    new DriveToPose(this,
                            new Pose(77, 55, this.drive.follower.getHeading()), 5)
            ).schedule();
        }
        // MOVE FORWARD
        else {
            new SequentialCommandGroup(
                    new DriveFwdByDist(this,24,20 )
            ).schedule();
        }
    }

}