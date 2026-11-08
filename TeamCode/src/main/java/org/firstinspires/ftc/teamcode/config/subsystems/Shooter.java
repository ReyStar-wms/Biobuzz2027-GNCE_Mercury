package org.firstinspires.ftc.teamcode.config.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.config.core.SubsysCore;
import org.firstinspires.ftc.teamcode.config.hardware.CachedServo;
import org.firstinspires.ftc.teamcode.config.core.util.ShotPlanner;
import org.firstinspires.ftc.teamcode.config.core.util.hardware.VoltageCompensatedMotorGroup;

@Configurable
public class Shooter extends SubsysCore {
    private final VoltageCompensatedMotorGroup flywheel;
    private final Motor.Encoder encoder;
    private final CachedServo hood;

    public static double INACTIVE_VELOCITY = 0.0;
    public static double VELOCITY_READY_THRESHOLD = 60.0;

    public static double kp = 0.015; // 0.01
    public static double ki = 0.0;
    public static double kd = 0;
    public static double kV = 0.000388; // 0.000455
    public static long VOLTAGE_CACHE_MS = 300;

    // Flywheel anticipation: feedforward proportional to how fast the *target* RPM is changing
    // (because the robot is moving toward/away from the goal), so the slow wheel tracks the moving
    // target instead of lagging behind it. It is added to the control output, not the setpoint, so
    // it is exactly zero at steady state and never biases a held shot. Tune FLYWHEEL_ACCEL_KFF up
    // from 0 on the field until the wheel follows distance changes without overshoot/oscillation.
    public static double FLYWHEEL_ACCEL_KFF = 0.0;       // motor power per (rpm/sec) of target change
    public static double FLYWHEEL_RATE_SMOOTHING = 0.3;  // EMA alpha for the target-rate estimate
    public static double FLYWHEEL_MAX_TARGET_RATE = 8000.0; // rpm/sec clamp on the rate estimate

    private double currentTargetVelocity = 0.0;
    private double currentTargetHoodAngle = 30;

    public static double MIN_HOOD_ANGLE_DEG = 30;
    public static double MAX_HOOD_ANGLE_DEG = 55.5;

    public static double HOOD_MAX_SERVO_POS = 1;
    public static double HOOD_GEAR_RATIO = (double) 300 / 44;
    public static double HOOD_SERVO_RANGE = 255;
    public static double IDLE_HOOD_ANGLE_DEG = 31;

    PIDFController pidfController;
    boolean active = false;

    private final ElapsedTime ffTimer = new ElapsedTime();
    private double prevTargetVelocity = 0.0;
    private double filteredTargetRate = 0.0;

    public static boolean testModeOnly = false;

    private double plannedDistPoseUnits = 0.0;        // inches
    private double plannedTargetRpm = 0.0;            // rpm
    private double plannedHoodBaselineDeg = IDLE_HOOD_ANGLE_DEG; // hood deg (your convention)
    private boolean shotPossible = false;

    // Spin-up override: when held (>= 0), the flywheel targets exactly this RPM instead of the
    // live distance-based target, so it stays at shot speed while driving in to collect (rather
    // than slowing as the robot nears the goal and re-spinning at the shot). Cleared (-1) means
    // follow the planned target. Set it to the RPM of the pose you'll shoot from.
    private double spinUpTargetRpm = -1.0;

    private boolean rpmClampEnabled = false;
    private double minClampedRpm = 0.0;
    private double maxClampedRpm = 2800.0;

    MotorEx m1, m2;

    public void setSpinUpRpm(double rpm) {
        spinUpTargetRpm = Math.max(0.0, rpm);
    }

    public void setRpmClamp(double minRpm, double maxRpm) {
        rpmClampEnabled = true;
        minClampedRpm = minRpm;
        maxClampedRpm = maxRpm;
    }

    public void clearRpmClamp() {
        rpmClampEnabled = false;
    }

    public void clearSpinUp() {
        spinUpTargetRpm = -1.0;
    }

    public boolean isSpinUpHeld() {
        return spinUpTargetRpm >= 0.0;
    }

    public void setPlannedShot(double distPoseUnits, double targetRpm, double hoodBaselineDegFromVertical, boolean plannedPossible) {
        plannedDistPoseUnits = distPoseUnits;
        plannedTargetRpm = targetRpm;
        plannedHoodBaselineDeg = hoodBaselineDegFromVertical;
        shotPossible = plannedPossible;
    }

    public static double NOMINAL_VOLTAGE = 11.8;

    public Shooter() {
        m1 = new MotorEx(h, "shooter1", Motor.GoBILDA.BARE);
        m2 = new MotorEx(h, "shooter2", Motor.GoBILDA.BARE);
        m2.setInverted(true);
        flywheel = new VoltageCompensatedMotorGroup(h, VOLTAGE_CACHE_MS, NOMINAL_VOLTAGE, m1, m2);
        flywheel.setRunMode(Motor.RunMode.RawPower);
        flywheel.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        encoder = m1.encoder;
        encoder.setDirection(Motor.Direction.FORWARD);

        hood = new CachedServo(h.get(Servo.class, "hood"));

        pidfController = new PIDFController(kp, ki, kd, kV);

        testModeOnly = false;
        spinUpTargetRpm = -1.0; // always start with no spin-up hold
        setDefaultCommand(new RunCommand(() -> active = true, this));
    }

    public void setActive(boolean active) {
        this.active = active;
    }

    public boolean isActive() {
        return active;
    }

    public double getTargetVelocity() {
        return currentTargetVelocity;
    }

    public boolean readyToShoot() {
        if (!active) return false;
        return shotPossible;
    }

    private double hoodAngleToServoPos(double hoodAngleDeg) {
        double clipped = Range.clip(hoodAngleDeg, MIN_HOOD_ANGLE_DEG, MAX_HOOD_ANGLE_DEG);
        double t = (MAX_HOOD_ANGLE_DEG - clipped);
        return HOOD_MAX_SERVO_POS - t * HOOD_GEAR_RATIO / HOOD_SERVO_RANGE;
    }

    public double getVelocity() {
        return -encoder.getCorrectedVelocity();
    }

    @Override
    public void periodic() {
        pidfController.setPIDF(kp, ki, kd, kV);

        if (active && !testModeOnly) {
            // While held, target the spin-up RPM exactly (keeps the wheel at shot speed while
            // driving to collect); otherwise follow the live distance-based target.
            double target = isSpinUpHeld() ? spinUpTargetRpm : plannedTargetRpm;

            if (rpmClampEnabled) {
                target = Range.clip(target, minClampedRpm, maxClampedRpm);
            }

            currentTargetVelocity = Range.clip(target, 0.0, 2800);
            currentTargetHoodAngle = Range.clip(plannedHoodBaselineDeg, MIN_HOOD_ANGLE_DEG, MAX_HOOD_ANGLE_DEG);
        } else {
            currentTargetVelocity = INACTIVE_VELOCITY;
            currentTargetHoodAngle = plannedHoodBaselineDeg;
        }

        // Anticipation feedforward: power proportional to the rate of change of the target RPM, so
        // the wheel pre-spins along distance changes. Zero at steady state -> no bias on a held shot.
        double dt = ffTimer.seconds();
        ffTimer.reset();
        double accelFF = 0.0;
        if (dt > 1e-3 && dt < 0.1) {
            double rawRate = (currentTargetVelocity - prevTargetVelocity) / dt;
            rawRate = Range.clip(rawRate, -FLYWHEEL_MAX_TARGET_RATE, FLYWHEEL_MAX_TARGET_RATE);
            filteredTargetRate += FLYWHEEL_RATE_SMOOTHING * (rawRate - filteredTargetRate);
            accelFF = FLYWHEEL_ACCEL_KFF * filteredTargetRate;
        }
        prevTargetVelocity = currentTargetVelocity;

        double targetPower = MathUtils.clamp(
                pidfController.calculate(getVelocity(), getTargetVelocity()) + accelFF, -1, 1);
        flywheel.set(targetPower);

        // Physics comp removed — hood angle from ShotPlanner is already physics-correct
        // double hoodCmdDeg = (active && !testModeOnly)
        //         ? physicsCompensatedHoodDeg(plannedDistPoseUnits, currentTargetVelocity, getVelocity(), currentTargetHoodAngle)
        //         : currentTargetHoodAngle;
        double hoodCmdDeg = currentTargetHoodAngle;

        double servoPos = hoodAngleToServoPos(hoodCmdDeg);
        hood.setPosition(shotPossible ? servoPos : hoodAngleToServoPos(IDLE_HOOD_ANGLE_DEG));

        // m1/m2.getCurrent() are non-bulk hub round-trips (the most expensive reads in this loop) and
        // feed telemetry only -- skip the whole block, and them, when telemetry is off.
        if(telemetryEnabled){
            t.addData("Shooter Cached Voltage", flywheel.getCachedVoltage());
            t.addData("Flywheel 1 Current", m1.getCurrent(CurrentUnit.AMPS));
            t.addData("Flywheel 2 Current", m2.getCurrent(CurrentUnit.AMPS));
            t.addData("Shooter Velocity Error", getTargetVelocity() - getVelocity());
            t.addData("Shooter Target Power", targetPower);
            t.addData("Shooter Accel FF", accelFF);
            t.addData("Shooter Target Rate (rpm/s)", filteredTargetRate);
            t.addData("Shooter Velocity", getVelocity());
            t.addData("Shooter Target Velocity", getTargetVelocity());
            t.addData("Shooter Spin-Up Target", isSpinUpHeld() ? spinUpTargetRpm : -1);
            t.addData("Shooter Ready", readyToShoot());
            t.addData("Shooter testModeOnly", testModeOnly);
            t.addData("Shooter Planned Dist (in)", plannedDistPoseUnits);
            t.addData("Shooter Hood Baseline (deg)", currentTargetHoodAngle);
            t.addData("Shooter Hood Cmd (deg)", hoodCmdDeg);
            t.addData("Shooter Commanded Servo Pos", servoPos);
        }
    }
}