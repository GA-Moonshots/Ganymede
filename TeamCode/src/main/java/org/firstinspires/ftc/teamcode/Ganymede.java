package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.Robot;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.Drive;
import org.firstinspires.ftc.teamcode.commands.DriveToBlue;
import org.firstinspires.ftc.teamcode.commands.FwdByDist;
import org.firstinspires.ftc.teamcode.subsystems.PedroDrive;
import org.firstinspires.ftc.teamcode.subsystems.Sensors;

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
    public boolean isLeft;

    // SUBSYSTEMS
    public PedroDrive drive;
    public Sensors sensors;

    // Convenience references
    public Telemetry telemetry;
    public HardwareMap hardwareMap;

    // Pedro Pathing specific
    public Pose startPose;

    /**
     * Constructor for TeleOp mode
     */
    public Ganymede(LinearOpMode opMode) {
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;
        this.hardwareMap = opMode.hardwareMap;

        // Initialize gamepads
        player1 = new GamepadEx(opMode.gamepad1);
        player2 = new GamepadEx(opMode.gamepad2);

        // Default starting pose for TeleOp (can be loaded from saved pose)
        startPose = new Pose(0, 0, 0); // Pedro uses degrees for heading

        initTeleOp();
    }

    /**
     * Constructor for Autonomous mode
     */
    public Ganymede(LinearOpMode opMode, boolean isRed, boolean isLeft) {
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;
        this.hardwareMap = opMode.hardwareMap;
        this.isRed = isRed;
        this.isLeft = isLeft;

        // Initialize gamepads (may not be used in auto but keeps consistency)
        player1 = new GamepadEx(opMode.gamepad1);
        player2 = new GamepadEx(opMode.gamepad2);

        // Set starting pose based on alliance and side
        if (isRed) {
            if (isLeft) {
                // Red Left starting position
                startPose = new Pose(0, 0, 0);
            } else {
                // Red Right starting position
                startPose = new Pose(0, 0, 0);
            }
        } else {
            if (isLeft) {
                // Blue Left starting position
                startPose = new Pose(0, 0, 0);
            } else {
                // Blue Right starting position
                startPose = new Pose(0, 0, 0);
            }
        }

        initAuto();
    }

    /**
     * Initialize robot for TeleOp
     */
    public void initTeleOp() {
        // Initialize drive subsystem with Pedro Pathing
        drive = new PedroDrive(this, startPose);

        // Initialize sensor package (mostly unchanged from your original)
        sensors = new Sensors(this);

        // Register subsystems with the command scheduler
        register(drive, sensors);

        // Set default commands
        drive.setDefaultCommand(new Drive(this));

        // Configure button bindings for TeleOp
        /*

                .__                                      ____
        ______  |  |  _____   ___.__.  ____ _______     /_   |
        \____ \ |  |  \__  \ <   |  |_/ __ \\_  __ \     |   |
        |  |_> >|  |__ / __ \_\___  |\  ___/ |  | \/     |   |
        |   __/ |____/(____  // ____| \___  >|__|        |___|
        |__|               \/ \/          \/

        */

        // Right bumper - Slow mode (handled in Drive command)
        // Already handled by checking gamepad1.right_bumper in Drive command

        // A Button - Reset heading
        new GamepadButton(player1, GamepadKeys.Button.A)
                .whenPressed(() -> drive.resetHeading());

        // B Button - Toggle field-centric mode
        new GamepadButton(player1, GamepadKeys.Button.B)
                .whenPressed(() -> drive.toggleFieldCentric());

        new GamepadButton(player1, GamepadKeys.Button.BACK)
                .whenPressed(new InstantCommand(() -> {
                    Pose redBaseZone = new Pose(40, 32, 0); // Center of red BASE, facing blue
                    drive.setPose(redBaseZone);
                    sensors.addTelemetry("✓ Relocalized", "Red BASE ZONE");
                }));

        new GamepadButton(player1, GamepadKeys.Button.X)
                .whenPressed(new DriveToBlue(this, 10000));

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

    }

    /**
     * Initialize robot for Autonomous
     */
    public void initAuto() {
        // Initialize drive with starting pose
        drive = new PedroDrive(this, startPose);

        // Initialize sensors with April Tag tracking enabled
        sensors = new Sensors(this);

        // Register subsystems
        register(drive, sensors);

        sensors.addTelemetry("Status", "Robot initialized for Autonomous");
        sensors.addTelemetry("Alliance", isRed ? "RED" : "BLUE");
        sensors.addTelemetry("Starting Side", isLeft ? "LEFT" : "RIGHT");
        sensors.addTelemetry("Start Pose", startPose.toString());

        // TODO: Call autonomous routines here

            new SequentialCommandGroup(
                new FwdByDist(this,12,10000000 )

            ).schedule();

    }

}