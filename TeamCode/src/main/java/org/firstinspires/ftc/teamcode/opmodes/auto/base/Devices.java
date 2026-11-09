package org.firstinspires.ftc.teamcode.opmodes.auto.base;

/**
 * Every name this robot expects to find in the Driver Station configuration, in one place.
 *
 * <p>These mirror the names the subsystems ask for in their constructors. If a check in this package
 * reports a device as missing, either the config on the Control Hub is wrong or a name here (and in
 * the matching subsystem) has drifted.
 */
public final class Devices {

    private Devices() {
    }

    // Drivetrain (also driven by the lift through the PTO clutch). Names come from
    // pedroPathing.Constants.driveConstants.
    public static final String DRIVE_FRONT_LEFT = "fl";
    public static final String DRIVE_BACK_LEFT = "bl";
    public static final String DRIVE_FRONT_RIGHT = "fr";
    public static final String DRIVE_BACK_RIGHT = "br";
    public static final String[] DRIVE_MOTORS = {
            DRIVE_FRONT_LEFT, DRIVE_BACK_LEFT, DRIVE_FRONT_RIGHT, DRIVE_BACK_RIGHT
    };

    /** goBILDA Pinpoint odometry computer (localizerConstants.hardwareMapName). */
    public static final String PINPOINT = "pp";

    // Turret: two geared servos driving one output, with an absolute analog encoder on it.
    public static final String TURRET_SERVO_1 = "ts1";
    public static final String TURRET_SERVO_2 = "ts2";
    public static final String TURRET_ENCODER = "te1";

    // Intake / transfer.
    public static final String INTAKE_MOTOR = "intakeMotor";
    public static final String TRANSFER_MOTOR = "transferMotor";
    public static final String INTAKE_PITCH_SERVO = "intakePitch";

    // Shooter.
    public static final String FLYWHEEL_1 = "shooter1";
    public static final String FLYWHEEL_2 = "shooter2";
    public static final String HOOD_SERVO = "hood";

    /** Gate in front of the flywheel. */
    public static final String DOOR_SERVO = "door";

    /** Lift power take-off clutch. */
    public static final String PTO_SERVO = "pto";

    // Storage: s1-s3 are the beam-break receivers (inputs), p1-p3 power their emitters (outputs).
    public static final String[] BEAM_RECEIVERS = {"s1", "s2", "s3"};
    public static final String[] BEAM_EMITTERS = {"p1", "p2", "p3"};

    public static final String LIMELIGHT = "Limelight";

    // Limelight pipeline indices, as switched in Limelight.periodic(). Kept here rather than
    // imported from the subsystem so the checks stay independent of the whole robot stack; they
    // must match Limelight's switchPipeline calls and Limelight.BALL_PIPELINE_INDEX.
    public static final int PIPELINE_MOTIF = 0;
    public static final int PIPELINE_LOCALIZATION = 1;
    public static final int PIPELINE_BALL_DETECTOR = 3;
}
