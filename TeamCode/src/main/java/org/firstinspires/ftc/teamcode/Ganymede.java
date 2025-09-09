package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.Robot;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.Drive;
import org.firstinspires.ftc.teamcode.subsystems.PedroDrive;
import org.firstinspires.ftc.teamcode.subsystems.SensorPackage;

/**
 * Main robot class for the 2025 FTC season using SolversLib and Pedro Pathing
 * This replaces the old Callisto.java and removes unnecessary subsystems
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
    public SensorPackage sensors;

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
        setAutonomousStartPose();

        initAuto();
    }

    /**
     * Initialize robot for TeleOp
     */
    public void initTeleOp() {
        // Initialize drive subsystem with Pedro Pathing
        drive = new PedroDrive(this, startPose);

        // Initialize sensor package (mostly unchanged from your original)
        sensors = new SensorPackage(this);

        // Register subsystems with the command scheduler
        register(drive, sensors);

        // Set default commands
        drive.setDefaultCommand(new Drive(this));

        // Configure button bindings for TeleOp
        configureButtonBindings();

        telemetry.addData("Status", "Robot initialized for TeleOp");
        telemetry.update();
    }

    /**
     * Initialize robot for Autonomous
     */
    public void initAuto() {
        // Initialize drive with starting pose
        drive = new PedroDrive(this, startPose);

        // Initialize sensors with April Tag tracking enabled
        sensors = new SensorPackage(this);
        sensors.enableAprilTagTracking();

        // Register subsystems
        register(drive, sensors);

        telemetry.addData("Status", "Robot initialized for Autonomous");
        telemetry.addData("Alliance", isRed ? "RED" : "BLUE");
        telemetry.addData("Starting Side", isLeft ? "LEFT" : "RIGHT");
        telemetry.addData("Start Pose", startPose.toString());
        telemetry.update();
    }

    /**
     * Configure button mappings for TeleOp
     */
    private void configureButtonBindings() {
        // --- PLAYER 1 CONTROLS (Drive) ---

        // Right bumper - Slow mode (handled in Drive command)
        // Already handled by checking gamepad1.right_bumper in Drive command

        // A Button - Reset heading
        new GamepadButton(player1, GamepadKeys.Button.A)
                .whenPressed(() -> drive.resetHeading());

        // B Button - Toggle field-centric mode
        new GamepadButton(player1, GamepadKeys.Button.B)
                .whenPressed(() -> drive.toggleFieldCentric());

        // X Button - Save current pose (for persistent pose between runs)
        new GamepadButton(player1, GamepadKeys.Button.X)
                .whenPressed(() -> drive.saveCurrentPose());

        // Y Button - Snap to nearest cardinal direction (0, 90, 180, 270)
        new GamepadButton(player1, GamepadKeys.Button.Y)
                .whenPressed(() -> drive.snapToCardinal());

        // --- PLAYER 2 CONTROLS ---
        // Add subsystem-specific controls here as you add them back
        // For now, keeping it minimal as requested

        // Dpad Up - Enable April Tag position correction
        new GamepadButton(player2, GamepadKeys.Button.DPAD_UP)
                .whenPressed(() -> sensors.enableAprilTagCorrection());

        // Dpad Down - Disable April Tag position correction
        new GamepadButton(player2, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(() -> sensors.disableAprilTagCorrection());
    }

    /**
     * Set the starting pose based on alliance and starting position
     */
    private void setAutonomousStartPose() {
        // These are example starting positions - adjust based on your field setup
        // Pedro uses inches for X/Y and degrees for heading

        if (isRed) {
            if (isLeft) {
                // Red Left starting position
                startPose = new Pose(-36, -62, 180);
            } else {
                // Red Right starting position
                startPose = new Pose(12, -62, 180);
            }
        } else {
            if (isLeft) {
                // Blue Left starting position
                startPose = new Pose(-12, 62, 0);
            } else {
                // Blue Right starting position
                startPose = new Pose(36, 62, 0);
            }
        }
    }

    /**
     * Update method to be called in OpMode loops
     * This ensures gamepad states are updated
     */
    public void update() {
        // Update gamepad states
        player1.readButtons();
        player2.readButtons();

        // Update Pedro's localization
        drive.update();

        // Update sensor readings if needed
        if (sensors.isAprilTagCorrectionEnabled()) {
            sensors.processAprilTags();
        }
    }

    /**
     * Emergency stop - disable all subsystems
     */
    public void emergencyStop() {
        drive.stop();
        telemetry.addData("Status", "EMERGENCY STOP ACTIVATED");
        telemetry.update();
    }
}