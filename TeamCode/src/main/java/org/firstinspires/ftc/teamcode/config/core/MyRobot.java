package org.firstinspires.ftc.teamcode.config.core;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.PanelsField;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.Robot;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.utils.LoopTimer;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.commands.OuttakeCommand;
import org.firstinspires.ftc.teamcode.config.commands.OuttakeCommandAuto;
import org.firstinspires.ftc.teamcode.config.commands.OuttakeCommandTele;
import org.firstinspires.ftc.teamcode.config.commands.TransferCommand;
import org.firstinspires.ftc.teamcode.config.core.util.Ball;
import org.firstinspires.ftc.teamcode.config.core.util.BallLocalizer;
import org.firstinspires.ftc.teamcode.config.core.util.BallPathPlanner;
import org.firstinspires.ftc.teamcode.config.core.util.ShotPlanner;
import org.firstinspires.ftc.teamcode.config.core.util.robothelper.Artifact;
import org.firstinspires.ftc.teamcode.config.core.util.robothelper.Motif;
import org.firstinspires.ftc.teamcode.config.core.util.robothelper.OpModeType;
import org.firstinspires.ftc.teamcode.config.core.util.SAT2D;
import org.firstinspires.ftc.teamcode.config.core.util.robothelper.SubsystemConfig;
import org.firstinspires.ftc.teamcode.config.core.util.hardware.ToggleButton;
import org.firstinspires.ftc.teamcode.config.core.util.hardware.RumbleManager;
import org.firstinspires.ftc.teamcode.config.core.util.VisionMeasurement;
import org.firstinspires.ftc.teamcode.config.subsystems.Door;
import org.firstinspires.ftc.teamcode.config.subsystems.Intake;
import org.firstinspires.ftc.teamcode.config.subsystems.Lift;
import org.firstinspires.ftc.teamcode.config.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.config.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.config.subsystems.Storage;
import org.firstinspires.ftc.teamcode.config.subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Arrays;
import java.util.List;

@Configurable
public class MyRobot extends Robot {
    /**
     * Behavioral modes that used to be separate subclasses. Pick one during init (cycle with g1
     * RIGHT_BUMPER) or set it up front with {@link #setRobotMode(RobotMode)} in your OpMode.
     * <ul>
     *   <li>{@link #BASE}        - plain teleop, no extra g1 DPAD_RIGHT binding.</li>
     *   <li>{@link #MANUAL_RPM}  - g1 LEFT_BUMPER toggles CLOSE/FAR flywheel RPM presets (selectable
     *       during init too) + g1 DPAD_RIGHT auto-park.</li>
     * </ul>
     */
    public enum RobotMode { MANUAL_RPM, BASE }

    HardwareMap h;
    public JoinedTelemetry t;
    public Follower f;
    List<LynxModule> hubs;
    LoopTimer lt;
    public Intake intake;
    public Turret turret;
    public Door door;
    public Storage storage;
    public Lift lift;
    public Limelight ll;
    public Shooter shooter;
    Timer runtime;
    public GamepadEx g1, g2;
    public static Pose autoEndPose;
    public static Motif currentMotif;
    public static int endTurretWrapCount;
    public static VisionMeasurement vm;
    public static Pose rawPinpointPose;
    public static double runtimeNow;
    Pose goalPose;
    OpModeType opModeType;

    public static Boolean isRed = null;
    public static double TURRET_OFFSET_INCHES = -0.905511811;

    public static double chassisLeft = 5.5196850394; // Inches
    public static double chassisRight = 5.5196850394;
    public static double chassisBack = 6.8661417323;
    public static double chassisFront = 10.413385827;

    public static double chassisLeftOut = 7.375;
    public static double chassisRightOut = 7.375;
    public static double chassisBackOut = 6.8661417323;
    public static double fieldWallThickness = 1.25;

    public static double fieldSize = 141.5;

    // ===== Mode selection (replaces the old MyRobot_AutoPark / MyRobot_ManualRPM subclasses) =====
    // static so the selection (set in auto / pre-match) carries straight into teleop unchanged.
    private static RobotMode robotMode = RobotMode.MANUAL_RPM;

    // ---- Auto-park config + state (part of MANUAL_RPM, triggered by g1 DPAD_RIGHT) ----
    // Park pose. RED uses PARK_X_RED; BLUE uses (fieldSize - PARK_X_RED). y/heading are shared.
    public static double PARK_X_RED = 38;
    public static double PARK_Y = 30;
    public static double PARK_HEADING_DEG = 315;
    // If any drive stick exceeds this while parking, the driver is taking over: cancel the park.
    public static double STICK_CANCEL_DEADBAND = 0.1;
    private boolean autoParking = false;

    // ---- MANUAL_RPM config + state (was MyRobot_ManualRPM) ----
    // Two manual flywheel RPM presets. Toggled with g1 LEFT_BUMPER; defaults to FAR.
    public static double CLOSE_MIN_RPM = 1400;
    public static double CLOSE_MAX_RPM = 1900;
    public static double FAR_MIN_RPM = 1850;
    public static double FAR_MAX_RPM = 2150;
    public static boolean farMode = true; // false = CLOSE, true = FAR (default); static so the selection carries across OpModes
    private boolean seeded  = false; // apply the selected baseline on the first teleop loop
    // =============================================================================================

    // ---- Telemetry mode ----
    // When OFF, the bulk telemetry is neither polled nor formatted (saving loop time); only a
    // one-line indicator plus loop time/frequency are shown so you can confirm the loop sped up.
    // Toggle with g2 OPTIONS during init OR the match; also live-editable in Panels (@Configurable).
    // static so the choice carries from auto into teleop, like the other mode flags above.
    public static boolean telemetryEnabled = true;
    private final ToggleButton telemetryButton = new ToggleButton(telemetryEnabled);

    SAT2D.PolygonSet launchZones = new SAT2D.PolygonSet()
            .add("Close",
                    new SAT2D.Point2(0, fieldSize),
                    new SAT2D.Point2(fieldSize, fieldSize),
                    new SAT2D.Point2(fieldSize/2, fieldSize/2))
            .add("Far",
                    new SAT2D.Point2(fieldSize/2, 24),
                    new SAT2D.Point2(fieldSize/2 - 24, 0),
                    new SAT2D.Point2(fieldSize/2 + 24, 0));

    ToggleButton autoFireButton, turretAlwaysReadyButton;
    public RumbleManager rumbleManager;
    OuttakeCommandTele OuttakeCommandTele;
    List<SubsystemConfig> subsysList;
    ShotPlanner planner;
    public BallLocalizer ballLocalizer;
    public BallPathPlanner ballPathPlanner;
    boolean [] enabledSubsys = new boolean[SubsystemConfig.values().length];
    int slotSelect = 0;

    public boolean hasSubsystem(SubsystemConfig subsystem){
        return enabledSubsys[subsystem.ordinal()];
    }

    public boolean hasSubsystems(List<SubsystemConfig> subsystems){
        for(SubsystemConfig subsystem: subsystems){
            if(!hasSubsystem(subsystem)) return false;
        }
        return true;
    }

    public MyRobot(HardwareMap h, Telemetry t, Gamepad g1, Gamepad g2, List<SubsystemConfig> subsysList, OpModeType opModeType){
        this.opModeType = opModeType;
        this.subsysList = subsysList;
        for(SubsystemConfig subsystem: subsysList){
            enabledSubsys[subsystem.ordinal()] = true;
        }

        PanelsField.INSTANCE.getField().setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
        PanelsField.INSTANCE.getField().setStyle("red", "blue", 2.0);

        this.h = h;
        this.t = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), t);
        hubs = this.h.getAll(LynxModule.class);
        for(LynxModule hub: hubs){
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        this.g1 = new GamepadEx(g1);
        this.g2 = new GamepadEx(g2);
        this.rumbleManager = new RumbleManager(this.g1.gamepad);

        SubsysCore.setGlobalParameters(this.h, this.t);

        if(hasSubsystem(SubsystemConfig.INTAKE)){
            this.intake = new Intake();
            this.storage = new Storage();
        }
        if(hasSubsystem(SubsystemConfig.LL)){
            this.ll = new Limelight();
        }
        if(hasSubsystem(SubsystemConfig.SHOOTER)) this.shooter = new Shooter();
        if(hasSubsystem(SubsystemConfig.DOOR)) this.door = new Door();
        if(hasSubsystem(SubsystemConfig.LIFT)) this.lift = new Lift();
        if(hasSubsystem(SubsystemConfig.FOLLOWER)){
            this.f = Constants.createFollower(this.h); // Put follower lower than limelight.

            if(hasSubsystem(SubsystemConfig.TURRET)){
                turret = new Turret(endTurretWrapCount);
                if(this.opModeType == OpModeType.TELEOP) turretAlwaysReadyButton = new ToggleButton(false);
            }
        }

        if(this.opModeType == OpModeType.TELEOP){
            if(hasSubsystems(Arrays.asList(SubsystemConfig.INTAKE, SubsystemConfig.DOOR, SubsystemConfig.SHOOTER, SubsystemConfig.TURRET))){
                this.autoFireButton = new ToggleButton(false);
            }
        }

        if(hasSubsystems(List.of(SubsystemConfig.INTAKE, SubsystemConfig.DOOR, SubsystemConfig.SHOOTER, SubsystemConfig.TURRET))){
            OuttakeCommandTele = new OuttakeCommandTele(intake, turret, shooter, door, storage, this::cueShotRumble);
        }

        if(isRed == null) isRed = false;
        this.lt = new LoopTimer();
        this.runtime = new Timer();
        this.planner = new ShotPlanner();
        ShotPlanner.tel = this.t;

        if(hasSubsystems(List.of(SubsystemConfig.LL, SubsystemConfig.FOLLOWER))){
            this.ballLocalizer = new BallLocalizer();
            this.ballPathPlanner = new BallPathPlanner();
        }

        if(hasSubsystem(SubsystemConfig.FOLLOWER)) {
            try {
                f.getPoseTracker().getLocalizer().resetIMU();
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }

            Constants.fusionLocalizer.t = this.t;
        }
    }
    public MyRobot(HardwareMap h, Telemetry t, Gamepad g1, Gamepad g2, List<SubsystemConfig> subsysList){
        this(h, t, g1, g2, subsysList, OpModeType.TELEOP);
    }

    public MyRobot(HardwareMap h, Telemetry t, Gamepad g1, Gamepad g2, OpModeType opModeType){
        this(h, t, g1, g2, Arrays.asList(SubsystemConfig.values()), opModeType);
    }

    // ===== Mode helpers =====

    /** Set the behavioral mode directly (e.g. from your OpMode's initialize()). */
    public void setRobotMode(RobotMode mode){ robotMode = mode; }

    /** Current behavioral mode. */
    public RobotMode getRobotMode(){ return robotMode; }

    /** Advance to the next mode (MANUAL_RPM -> BASE -> MANUAL_RPM ...). */
    private void cycleRobotMode(){
        RobotMode[] modes = RobotMode.values();
        robotMode = modes[(robotMode.ordinal() + 1) % modes.length];
    }

    private String manualRpmLabel(){
        return farMode
                ? ("FAR [" + FAR_MIN_RPM + "-" + FAR_MAX_RPM + "]")
                : ("CLOSE [" + CLOSE_MIN_RPM + "-" + CLOSE_MAX_RPM + "]");
    }

    /** Alliance, robot mode, and CLOSE/FAR RPM preset -- pinned to the top of match telemetry
     *  (called first thing in {@link #startPeriodic()}, before anything else is added, so it shows
     *  above everything in both auto and teleop). */
    public void addStatusTelemetry(){
        t.addData("Alliance", (isRed != null && isRed) ? "RED" : "BLUE");
        t.addData("Robot Mode", robotMode);
        t.addData("Manual RPM Mode", manualRpmLabel());
        t.addData("Telemetry Mode", telemetryEnabled ? "ON" : "OFF");
//        t.addData("RPM Preset", manualRpmLabel());
    }

    /** Read the telemetry-mode toggle (g2 OPTIONS) once per loop. Honors live Panels edits to
     *  {@link #telemetryEnabled} and flips it on a button press. Safe to call in init and the match. */
    private void updateTelemetryToggle(){
        telemetryButton.setVal(telemetryEnabled);                          // honor live Panels edits
        telemetryButton.input(g1.getButton(GamepadKeys.Button.OPTIONS));   // flips on press
        telemetryEnabled = telemetryButton.getVal();
        SubsysCore.telemetryEnabled = telemetryEnabled; // let subsystems skip telemetry-only hw reads
    }

    public void allianceSelection(){
        if(g1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) isRed = !isRed;
        t.addData("Current Alliance", isRed?"RED": "BLUE");
    }

    public void startDrive(){
        f.startTeleopDrive();
        f.update();
        runtime.resetTimer();
    }

    public void driveControls(){
        if(robotMode == RobotMode.MANUAL_RPM){
            autoParkDriveControls();
        } else {
            baseDriveControls();
        }
    }

    /** The original (non-park) driver bindings. */
    private void baseDriveControls(){
        if(hasSubsystem(SubsystemConfig.FOLLOWER)){
            f.setTeleOpDrive(g1.getLeftY(), -g1.getLeftX(), -g1.getRightX(), true);
        }

        if(g1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.8) cornerSquare();
        if(g1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) alignPreset();
        if(g1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) farWallSquare();
        if(g1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) allianceWallSquare();
    }

    /**
     * baseDriveControls plus a DPAD_RIGHT auto-park (used in MANUAL_RPM mode). While parking the
     * teleop drive call is suppressed so it doesn't fight the path; the park ends when the path
     * finishes or the driver moves a stick. The other g1 bindings are skipped while parking so a
     * stray press can't interrupt it.
     */
    private void autoParkDriveControls(){
        boolean justStarted = false;

        if(hasSubsystem(SubsystemConfig.FOLLOWER)){
            // Start auto-park on DPAD_RIGHT.
            if(g1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)){
                startAutoPark();
                justStarted = true;
            }

            // While parking (and not on the very loop we started), end it if the driver grabs the
            // sticks or the path completes, then hand control back to teleop drive.
            if(autoParking && !justStarted){
                boolean driverInput =
                        Math.abs(g1.getLeftY())  > STICK_CANCEL_DEADBAND ||
                                Math.abs(g1.getLeftX())  > STICK_CANCEL_DEADBAND ||
                                Math.abs(g1.getRightX()) > STICK_CANCEL_DEADBAND;
                if(driverInput || !f.isBusy()){
                    autoParking = false;
                    f.startTeleopDrive(); // resume normal driver control
                }
            }

            // Only drive from the sticks when NOT parking, otherwise we'd override the path.
            if(!autoParking){
                f.setTeleOpDrive(g1.getLeftY(), -g1.getLeftX(), -g1.getRightX(), true);
            }
        }

        // Existing g1 bindings, unchanged. Skipped while parking.
        if(!autoParking){
            if(g1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.8) cornerSquare();
            if(g1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) alignPreset();
            if(g1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) farWallSquare();
            if(g1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) allianceWallSquare();
        }

        if(telemetryEnabled) t.addData("Auto Parking", autoParking);
    }

    /** Alliance-correct park pose. Only x flips between alliances; y and heading are shared. */
    public Pose autoParkPose(){
        double x = (isRed != null && isRed) ? PARK_X_RED : (fieldSize - PARK_X_RED);
        return new Pose(x, PARK_Y, Math.toRadians(PARK_HEADING_DEG));
    }

    /** Build a straight line from the current pose to the park pose and start following it. */
    private void startAutoPark(){
        if(!hasSubsystem(SubsystemConfig.FOLLOWER)) return;
        Pose target = autoParkPose();
        f.followPath(
                f.pathBuilder()
                        .addPath(new BezierLine(f.getPose(), target))
                        .setLinearHeadingInterpolation(f.getHeading(), target.getHeading())
                        .build(),
                true); // holdEnd: keep holding the park pose once arrived
        autoParking = true;
    }


    public static double blueGoalPoseX = 5;
    public static double blueGoalPoseY = 137;
    private Pose blueGoalPose = new Pose(blueGoalPoseX, blueGoalPoseY);
    public void startInitLoop(){
        lt.start();
        resetCache();
        allianceSelection();

        // Cycle the behavioral mode pre-match with g1 RIGHT_BUMPER (RIGHT_BUMPER only fires the
        // shooter during the match, so there's no phase conflict).
        if(g1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) cycleRobotMode();
        t.addData("Robot Mode (RIGHT_BUMPER to cycle)", robotMode);

        // MANUAL_RPM: let the driver preselect CLOSE/FAR with LEFT_BUMPER (same button as in-match).
        // No flywheel motion happens during init -- this only selects the mode the first teleop
        // loop will apply.
        if(robotMode == RobotMode.MANUAL_RPM){
            if(g1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) farMode = !farMode;
            t.addData("Manual RPM Mode (LEFT_BUMPER to toggle)", manualRpmLabel());
        }

        // Let the driver pre-select telemetry mode during init; carries into the match via the static.
        updateTelemetryToggle();
        t.addData("Telemetry (g1 OPTIONS to toggle)", telemetryEnabled ? "ON" : "OFF");

        g1.readButtons();
        g2.readButtons();

        if(!isRed) goalPose = blueGoalPose;
        else goalPose = blueGoalPose.mirror(fieldSize);
    }

    public void endInitLoop(){
        lt.end();
        if(hasSubsystem(SubsystemConfig.FOLLOWER)) t.addData("Current Pose", f.getPose());
        t.addData("Loop Time (ms)", lt.getMs());
        t.addData("Loop Frequency (Hz)", lt.getHz());
        t.update();
    }

    public void stop(){
        if(hasSubsystem(SubsystemConfig.FOLLOWER)) autoEndPose = f.getPose();
        if(hasSubsystem(SubsystemConfig.TURRET)) endTurretWrapCount = turret.getWrapCount();
    }

    public void resetCache(){
        for(LynxModule hub: hubs){
            hub.clearBulkCache();
        }
    }

    String prevZone = null;

    public void startPeriodic(){
        updateTelemetryToggle(); // g2 OPTIONS toggles telemetry mode (also editable in Panels)
        addStatusTelemetry(); // pin Alliance / Robot Mode / RPM preset / Telemetry Mode to the very top (always shown)

        blueGoalPose = new Pose(blueGoalPoseX, blueGoalPoseY);
        if(!isRed) goalPose = blueGoalPose;
        else goalPose = blueGoalPose.mirror(fieldSize);

        lt.start();
        resetCache();
        if(hasSubsystem(SubsystemConfig.FOLLOWER)){
            Pose turretPose = new Pose(
                    f.getPose().getX() + TURRET_OFFSET_INCHES * Math.cos(f.getPose().getHeading()),
                    f.getPose().getY() + TURRET_OFFSET_INCHES * Math.sin(f.getPose().getHeading()),
                    f.getPose().getHeading());

            if(hasSubsystem(SubsystemConfig.SHOOTER)) {
                // SOTM only while g1 holds the left trigger (teleop only); off otherwise.
                if(opModeType == OpModeType.TELEOP){
                    if(g1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5){
                        ShotPlanner.enableSOTM();
                        ShotPlanner.enableFPP();
                    } else {
                        ShotPlanner.disableSOTM();
                        ShotPlanner.disableFPP();
                    }
                }
                ShotPlanner.ShotCommand cmd = planner.plan(turretPose, f.getVelocity().getXComponent(), f.getVelocity().getYComponent(), f.getAngularVelocity(), goalPose, shooter.getVelocity());

                // ---- SOTM DEBUG TELEMETRY (remove when done tuning) ----
                if(telemetryEnabled){
                    t.addData("SOTM Virtual Goal X", cmd.virtualGoal.getX());
                    t.addData("SOTM Virtual Goal Y", cmd.virtualGoal.getY());
                    t.addData("SOTM Real Goal X", goalPose.getX());
                    t.addData("SOTM Real Goal Y", goalPose.getY());
                    t.addData("SOTM Flight Time (s)", String.format("%.3f", cmd.flightTimeSec));
                    t.addData("SOTM Lead X (in)", String.format("%.2f", cmd.virtualGoal.getX() - goalPose.getX()));
                    t.addData("SOTM Lead Y (in)", String.format("%.2f", cmd.virtualGoal.getY() - goalPose.getY()));
                    // Tuning: compare the lag-compensated launch velocity (what the lead is built from)
                    // against the raw measured velocity. Near a braked stop the launch value should fall
                    // to ~0 while the raw reading still lags backward -- if it doesn't, raise VELOCITY_LATENCY_SEC.
                    t.addData("SOTM Launch Vel (in/s)", String.format("%.1f, %.1f", cmd.launchVxFieldPosePerSec, cmd.launchVyFieldPosePerSec));
                    t.addData("SOTM Raw Vel (in/s)", String.format("%.1f, %.1f", f.getVelocity().getXComponent(), f.getVelocity().getYComponent()));
                    // Estimated accel feeding the prediction. During a decel this MUST read strongly
                    // opposite to velocity; if it reads ~0 or same-sign-as-velocity, the estimate is too
                    // laggy and the lead/turret over-extrapolate (shoots to the side). Compare vs Pedro's
                    // fresh one-step accel.
                    t.addData("SOTM Est Accel (in/s2)", String.format("%.0f, %.0f", cmd.robotAxFieldPosePerSec2, cmd.robotAyFieldPosePerSec2));
                    t.addData("SOTM Pedro Accel (in/s2)", String.format("%.0f, %.0f", f.getAcceleration().getXComponent(), f.getAcceleration().getYComponent()));
                }
// ---- END SOTM DEBUG TELEMETRY ----

                if (hasSubsystem(SubsystemConfig.TURRET))
                    // Aim from the turret-latency-led pose so the slow servo is settled by release.
                    turret.input(cmd.predictedTurretPose, cmd.virtualGoal);
                if (hasSubsystem(SubsystemConfig.SHOOTER))
                    shooter.setPlannedShot(cmd.distancePoseUnits, cmd.targetRpm, cmd.hoodBaselineDegFromVertical, cmd.possible);
            }

            if(hasSubsystem(SubsystemConfig.LL)){
                // TODO: make this local in ll class.
                runtimeNow = runtime.getElapsedTimeSeconds();
                if(ll.getMode() == Limelight.Mode.LOCALIZATION) {
                    ll.update(runtimeNow, Math.toDegrees(Constants.fusionLocalizer.getDeadReckoningPose().getHeading()));
                } else if(ll.getMode() == Limelight.Mode.BALL_DETECTION && ballLocalizer != null) {
                    // Project this frame's detections to the field and update each ball's velocity.
                    ballLocalizer.update(ll.getDetections(), f.getPose(), runtimeNow);
                    if(telemetryEnabled) t.addData("Tracked Balls", ballLocalizer.getBalls().size());
                }
            }

            if(opModeType == OpModeType.TELEOP) {
                if (hasSubsystems(List.of(SubsystemConfig.INTAKE, SubsystemConfig.TURRET, SubsystemConfig.SHOOTER, SubsystemConfig.DOOR, SubsystemConfig.FOLLOWER))){
                    autoFireButton.input(g1.getButton(GamepadKeys.Button.CROSS));
                    String zone = launchZones.firstHitName(chassisBox());
                    if (autoFireButton.getVal()) {
                        if (zone != null && !zone.equals(prevZone) && !OuttakeCommandTele.isScheduled() && storage.getSize() > 0) {
                            OuttakeCommandTele.schedule();
                        }
                        if (zone == null && OuttakeCommandTele.isScheduled())
                            CommandScheduler.getInstance().cancel(OuttakeCommandTele);
                    }
                    if(telemetryEnabled) t.addData("Current Zone", zone);
                    prevZone = zone;
                }

                if (hasSubsystem(SubsystemConfig.TURRET)) {
                    turretAlwaysReadyButton.input(g1.getButton(GamepadKeys.Button.CIRCLE));
                    turret.setAlwaysAtTarget(turretAlwaysReadyButton.getVal());
                }
            }
        }
        if(hasSubsystem(SubsystemConfig.INTAKE)){
            // Storage ignores current/velocity (both fields are dead), so don't pay for the motor
            // current read here -- getCurrent() is a separate hub round-trip, not a bulk read.
            storage.input(0, 0);
            intake.inputStorageSize(storage.getSize());
            if(hasSubsystem(SubsystemConfig.DOOR)) door.inputSize(storage.getSize());

            // Rumble: sustained while the magazine is full, preempted by a timed shot cue. The manager
            // edge-triggers the actual DS command, so this stays one cheap call per loop.
            rumbleManager.update(storage.getSize() == 3);
        }

        if(hasSubsystems(List.of(SubsystemConfig.INTAKE, SubsystemConfig.TURRET, SubsystemConfig.SHOOTER, SubsystemConfig.LIFT))){
            liftTeleop();
        }
    }
    public void runIntakeTeleop(){
        // LEFT_BUMPER is the CLOSE/FAR toggle in MANUAL_RPM, so don't also bind intake-reverse to
        // it there -- the toggle overrides it. In every other mode LEFT_BUMPER still reverses intake.
        if(robotMode == RobotMode.MANUAL_RPM) return;
        g1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenHeld(new InstantCommand(() -> intake.setMode(Intake.Mode.REVERSE), intake));
    }

    public void runTransferTeleop(){
        if (g1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER) && storage.getSize() != 0)
            schedule(new TransferCommand(intake, storage));
    }

    public void runShootTeleop(){
        double lt = g2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        double rt = g2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);

        if(lt > 0.1 || rt > 0.1) Turret.MANUAL_OFFSET += (rt - lt)/2;

        // MANUAL_RPM mode: g1 LEFT_BUMPER toggles CLOSE/FAR, applied as the shooter's spin-up hold
        // (latching, so one call per change is enough). Carries over whatever was selected during
        // init. Firing is still gated on shooter.readyToShoot() via OuttakeCommandTele.
        if(robotMode == RobotMode.MANUAL_RPM && hasSubsystem(SubsystemConfig.SHOOTER)){
            boolean toggled = g1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER);
            if(toggled) farMode = !farMode;

            // Seed the selected preset on the first loop, then re-apply only when it flips.
            if (toggled || !seeded) {
                if (farMode) shooter.setRpmClamp(FAR_MIN_RPM, FAR_MAX_RPM);
                else         shooter.setRpmClamp(CLOSE_MIN_RPM, CLOSE_MAX_RPM);
                seeded = true;
            }
//            t.addData("Manual RPM Mode", manualRpmLabel());
        }

        if (g1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) //  && storage.getSize() != 0
            schedule(OuttakeCommandTele);
    }


    Pose parkPose = new Pose(107.8, 28, Math.toRadians(-45));
    public void liftTeleop(){
        if(g1.wasJustPressed(GamepadKeys.Button.SQUARE)){
            turret.setTarget(Turret.Target.DISABLE);
            schedule(new RunCommand(() -> intake.setMode(Intake.Mode.DISABLE)));
            schedule(new RunCommand(() -> shooter.setActive(false), shooter));
        }
        if(g1.wasJustPressed(GamepadKeys.Button.TRIANGLE)) schedule(lift.FullLiftCommand());
        // if(g1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) schedule(new LiftCommand(intake, turret, shooter, lift));
    }

    public void endPeriodic() {
        if(hasSubsystem(SubsystemConfig.FOLLOWER)) f.update();
        this.run();
        if(telemetryEnabled){
            if (hasSubsystem(SubsystemConfig.FOLLOWER)) {
                t.addData("Current Pose", f.getPose());
                t.addData("Current Pinpoint Pose", Constants.fusionLocalizer.getDeadReckoningPose());
                t.addData("Current Velocity", f.getVelocity());
                t.addData("Follower Busy?" , f.isBusy());

                PanelsField.INSTANCE.getField().moveCursor(f.getPose().getX(), f.getPose().getY());
                PanelsField.INSTANCE.getField().circle(1);
                PanelsField.INSTANCE.getField().update();
            }
            if(hasSubsystems(List.of(SubsystemConfig.SHOOTER, SubsystemConfig.TURRET, SubsystemConfig.INTAKE, SubsystemConfig.DOOR))){
                t.addLine();
                t.addLine("SHOOTER DATA");
                if(!shooter.readyToShoot()) t.addLine("Shooter Not Ready");
                if(!turret.reachedTarget()) t.addLine("Turret Not Ready");
                if(!door.isOpen()) t.addLine("Door Not Open");
                t.addData("Shoot Ready?", shooter.readyToShoot() && turret.reachedTarget() && door.isOpen());
                t.addLine();
            }
            if(OuttakeCommandTele != null){
                t.addData("Outtake Command Scheduled", OuttakeCommandTele.isScheduled());
                t.addData("Outtake Command Done", OuttakeCommandTele.isFinished());
            }
        }
        g1.readButtons();
        g2.readButtons();
        lt.end();
        if(telemetryEnabled) t.addData("Current Motif", currentMotif == null ? "Not Detected" : currentMotif.name());
        t.addData("Loop Time (ms)", lt.getMs());
        t.addData("Loop Frequency (Hz)", lt.getHz());
        t.update();
    }

    public void overrideAutoEndPose(Pose newAutoEndPose){
        autoEndPose = newAutoEndPose;
    }

    public void onStart(){
        if(hasSubsystem(SubsystemConfig.FOLLOWER)){
            if(autoEndPose == null) autoEndPose = new Pose(chassisLeftOut, chassisBack, Math.toRadians(90));
            this.f.setStartingPose(autoEndPose);
            this.f.update();
            if(opModeType == OpModeType.TELEOP) this.startDrive();
        }
    }

    public void setPose(Pose pose){
        Turret.MANUAL_OFFSET = 0;
        f.setPose(pose);
        f.update();
    }

//    public void cornerSquare(){
//        if(isRed) setPose(new Pose(chassisLeftOut, chassisBack, Math.toRadians(90)));
//        else setPose(new Pose(fieldSize - chassisRightOut, chassisBack, Math.toRadians(90)));
//    }

    public void cornerSquare(){
        if(isRed) setPose(new Pose(10.346, 7.145, Math.toRadians(180)));
        else setPose(new Pose(fieldSize - 10.346, 7.145, Math.toRadians(0)));
    }


    public void farWallSquare(){
        setPose(new Pose(f.getPose().getX(), f.getPose().getY(), Math.toRadians(90)));
    }

    public void allianceWallSquare() {
        if(isRed) setPose(new Pose(f.getPose().getX(), f.getPose().getY(), Math.toRadians(0)));
        else setPose(new Pose(f.getPose().getX(), f.getPose().getY(), Math.toRadians(180)));
    }

    public void closeWallSquare(){
        setPose(new Pose(f.getPose().getX(), f.getPose().getY(), Math.toRadians(-90)));
    }

    // DPAD_UP alignment preset (blue-alliance coordinates). For RED, x mirrors to 144 - x.
    public static double alignPresetX = 18.415;
    public static double alignPresetY = 119.101;
    public static double alignPresetHeadingDeg = 141.21;
    Pose goalAlignPose = new Pose(18.415, 119.101, Math.toRadians(141.21));

    /** Snap the robot pose to the fixed alignment preset above (bound to DPAD_UP in teleop).
     *  RED mirrors x across the field (144 - x); y and heading are left as-is. */
    public void alignPreset(){
        setPose(isRed ? goalAlignPose.mirror() : goalAlignPose);
    }

    private SAT2D.ConvexPolygon chassisBox(){
        final double x = f.getPose().getX();
        final double y = f.getPose().getY();
        final double h = f.getPose().getHeading();

        final double c = Math.cos(h);
        final double s = Math.sin(h);

        // Local corners (robot frame)
        final double[] lx = { +chassisFront, +chassisFront, -chassisBack,  -chassisBack  };
        final double[] ly = { +chassisLeft,  -chassisRight, -chassisRight, +chassisLeft };

        SAT2D.Point2 p0 = rotTrans(lx[0], ly[0], x, y, c, s);
        SAT2D.Point2 p1 = rotTrans(lx[1], ly[1], x, y, c, s);
        SAT2D.Point2 p2 = rotTrans(lx[2], ly[2], x, y, c, s);
        SAT2D.Point2 p3 = rotTrans(lx[3], ly[3], x, y, c, s);

        return new SAT2D.ConvexPolygon(p0, p1, p2, p3);
    }

    private static SAT2D.Point2 rotTrans(double lx, double ly, double x, double y, double c, double s) {
        double X = x + c * lx - s * ly;
        double Y = y + s * lx + c * ly;
        return new SAT2D.Point2(X, Y);
    }


    /// Commands ///

    public Command shootAll(){
        return new OuttakeCommandTele(intake, turret, shooter, door, storage, this::cueShotRumble);
    }

    /** Shot-launch cue handed to the outtake command; fired when the transfer pushes the balls out.
     *  Null-safe so partial-robot configs without a rumble manager are fine. */
    private void cueShotRumble(){
        if(rumbleManager != null) rumbleManager.triggerShot();
    }

    public Command shootAll2(){
        return new OuttakeCommandAuto(intake, turret, shooter, door, storage);
    }

    public Command goToLinear(Pose tar){
        return new SequentialCommandGroup(
                new InstantCommand(() -> f.followPath(
                        f.pathBuilder()
                                .addPath(new BezierLine(f.getPose(), tar))
                                .setLinearHeadingInterpolation(f.getHeading(), tar.getHeading())
                                .build()
                )),
                new WaitUntilCommand(() -> !f.isBusy())
        );
    }
    /** Put the Limelight into neural ball-detection mode (so the localizer gets fed). */
    public void enableBallDetection(){
        if(hasSubsystem(SubsystemConfig.LL)) ll.setMode(Limelight.Mode.BALL_DETECTION);
    }

    /**
     * Dump every tracked ball (field position, velocity, confidence) to telemetry, and plot
     * each one on the Panels field. Call from an OpMode's {@code run()}; the localizer is
     * already updated each loop in {@code startPeriodic} when the LL is in ball-detection mode.
     */
    public void ballTelemetry(){
        if(ballLocalizer == null){
            t.addLine("Ball telemetry needs LL + FOLLOWER subsystems");
            return;
        }
        List<Ball> balls = ballLocalizer.getBalls();
        Pose camPose = hasSubsystem(SubsystemConfig.FOLLOWER) ? ballLocalizer.cameraOrigin(f.getPose()) : null;
        t.addLine();
        t.addData("Tracked Ball Count", balls.size());
        for(Ball b : balls){
            // camDist is the field distance from the lens to the ball — compare to a tape
            // measure to calibrate CAMERA_HEIGHT_IN / CAMERA_PITCH_DEG_DOWN.
            String camDist = camPose != null ? String.format(" | camDist %.1f in", camPose.distanceFrom(b.position)) : "";
            t.addData("Ball " + b.id + " (" + b.color + ")",
                    String.format("pos (%.1f, %.1f) in | vel (%.1f, %.1f) = %.1f in/s | conf %.2f%s",
                            b.position.getX(), b.position.getY(),
                            b.velocity.getX(), b.velocity.getY(), b.speed(), b.confidence, camDist));
            if(hasSubsystem(SubsystemConfig.FOLLOWER)){
                // Drawn into the same field buffer endPeriodic() flushes with update().
                PanelsField.INSTANCE.getField().moveCursor(b.position.getX(), b.position.getY());
                PanelsField.INSTANCE.getField().circle(2);
            }
        }
    }

    // Where to drive when no balls are detected: the human player end, where artifacts get
    // fed in. BLUE values matching AutoPaths' HP_END; mirrored for RED the same way AutoPaths
    // mirrors (blue.mirror()), so it lines up with the Far Vision + Far Spike auto.
    public static double fallbackCollectX = 11;
    public static double fallbackCollectY = 8.969;
    public static double fallbackCollectHeadingDeg = 180;
    /** If the robot is already within this x-distance of the human player end, skip the top-up
     *  trip there — it effectively drove to that wall while collecting and should just head back
     *  to shoot. */
    public static double SKIP_TOPUP_X_DIST_IN = 5.0;

    /** Fallback ball-collection pose (human player end) for the current alliance. */
    public Pose fallbackCollectPose(){
        Pose blue = new Pose(fallbackCollectX, fallbackCollectY, Math.toRadians(fallbackCollectHeadingDeg));
        return (isRed != null && isRed) ? blue.mirror() : blue;
    }

    /** Flywheel RPM needed to shoot from {@code shootPose} (distance to the current goal). */
    public double shooterRpmForPose(Pose shootPose){
        Pose g = goalPose != null ? goalPose : blueGoalPose;
        return planner.getLutRpm(shootPose.distanceFrom(g));
    }

    /**
     * Hold the flywheel at the RPM for {@code shootPose} so it does not slow down while driving in
     * to collect (which moves closer to the goal and would otherwise drop the target RPM and force
     * a re-spin at the shot). While held this is the target outright, until {@link #clearShooterSpinUp()}.
     * Set it to the pose you will shoot from next.
     */
    public Command spinUpShooterFor(Pose shootPose){
        return new InstantCommand(() -> {
            if(hasSubsystem(SubsystemConfig.SHOOTER)) shooter.setSpinUpRpm(shooterRpmForPose(shootPose));
        });
    }

    /** Release the spin-up hold (let the flywheel follow the live distance target again). */
    public Command clearShooterSpinUp(){
        return new InstantCommand(() -> {
            if(hasSubsystem(SubsystemConfig.SHOOTER)) shooter.clearSpinUp();
        });
    }

    /**
     * Scan only (instant): turn the Limelight on, aim it at the balls, and clear stale
     * tracks. The localizer then accumulates detections every loop in {@code startPeriodic}
     * for as long as the LL stays in ball-detection mode. Schedule this when you START a shot
     * so detections pile up <i>during</i> the shot, then call {@link #collectThreeBallsNoScan}.
     */
    public Command scanForBalls(){
        if(ballLocalizer == null){
            return new InstantCommand(() -> t.addLine("scanForBalls: LL + FOLLOWER required"));
        }
        return new InstantCommand(() -> {
            ballLocalizer.reset();
            ll.turnOn();
            ll.setMode(Limelight.Mode.BALL_DETECTION);
        });
    }

    /**
     * Plan the optimal route through three balls (accounting for velocity) and drive it,
     * using whatever the localizer has already gathered — <b>no scan wait</b>. Pair with a
     * prior {@link #scanForBalls()} (e.g. run during your shot). If fewer than
     * {@link BallPathPlanner#MAX_BALLS_IN_PATH} balls are seen (including none), it drives the
     * partial ball path and then continues straight to the HP-end wall (the x of
     * {@link #fallbackCollectPose()}, keeping the current y) to top up, in case vision missed
     * some — unless it is already at that wall. Turns the Limelight off when done.
     *
     * @param color only target this artifact color; pass {@code null} or {@link Artifact#NONE} for any
     */
    public Command collectThreeBallsNoScan(Artifact color){
        if(ballLocalizer == null || ballPathPlanner == null){
            // Needs both LL and FOLLOWER; do nothing rather than NPE if they aren't enabled.
            return new InstantCommand(() -> t.addLine("collectThreeBalls: LL + FOLLOWER required"));
        }
        final int[] expected = {0}; // balls the planned route expects to collect
        return new SequentialCommandGroup(
                // 1. Plan the route through the visible balls and start following it (if any).
                new InstantCommand(() -> {
                    BallPathPlanner.Plan plan = ballPathPlanner.plan(f, f.getPose(), ballLocalizer.getBalls(), color);
                    if (plan != null) {
                        expected[0] = plan.targets.size();
                        t.addData("Ball Path Targets", expected[0]);
                        t.addData("Ball Path Est Time (s)", String.format("%.2f", plan.estTimeSec));
                        f.followPath(plan.path, true);
                    } else {
                        expected[0] = 0;
                        t.addLine("No balls found");
                    }
                }),
                // 2. Drive the ball path (if one was set).
                new WaitUntilCommand(() -> !f.isBusy()),
                // 3. If fewer than a full load was visible, top up at the human player end --
                //    unless we already drove to the far end of the field (then just come back).
                new InstantCommand(() -> {
                    if (expected[0] >= BallPathPlanner.MAX_BALLS_IN_PATH) return;
                    Pose fb = fallbackCollectPose();
                    if (Math.abs(f.getPose().getX() - fb.getX()) < SKIP_TOPUP_X_DIST_IN) {
                        // Already at the HP-end wall; the explicit top-up drive would be redundant.
                        // TODO: a blind horizontal sweep here could still scoop nearby missed balls.
                        t.addData("Saw " + expected[0] + " and already at HP-end wall - skipping top-up", f.getPose());
                        return;
                    }
                    // Drive straight to the HP-end wall (its x) while staying at the current y.
                    Pose target = new Pose(fb.getX(), f.getPose().getY(), fb.getHeading());
                    t.addData("Only saw " + expected[0] + " - driving to wall to top up", target);
                    f.followPath(
                            f.pathBuilder()
                                    .addPath(new BezierLine(f.getPose(), target))
                                    .setLinearHeadingInterpolation(f.getHeading(), target.getHeading())
                                    .build(),
                            true);
                }),
                // 4. Drive the top-up path (immediate if none was set).
                new WaitUntilCommand(() -> !f.isBusy()),
                // 5. Turn the Limelight off until the next scan.
                new InstantCommand(() -> ll.turnOff())
        );
    }

    /** Plan + drive through three balls of any color, no scan wait. */
    public Command collectThreeBallsNoScan(){
        return collectThreeBallsNoScan(null);
    }

    /** Convenience: scan then collect, all in one (scan time is NOT hidden). */
    public Command collectThreeBalls(Artifact color){
        return new SequentialCommandGroup(scanForBalls(), collectThreeBallsNoScan(color));
    }

    /** Collect three balls of any color (scan + collect). */
    public Command collectThreeBalls(){
        return collectThreeBalls(null);
    }

    public Command goToTangential(Pose tar){
        return new SequentialCommandGroup(
                new InstantCommand(() -> f.followPath(
                        f.pathBuilder()
                                .addPath(new BezierLine(f.getPose(), tar))
                                .setTangentHeadingInterpolation()
                                .build()
                )),
                new WaitUntilCommand(() -> !f.isBusy())
        );
    }
}