package org.firstinspires.ftc.teamcode.config.core.util;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.util.InterpLUT;
import com.seattlesolvers.solverslib.util.MathUtils;

import java.util.ArrayDeque;

@Configurable
public class ShotPlanner {

    public static class ShotCommand {
        /** Robot pose predicted to the release instant (now + release latency). The shot geometry
         *  -- distance, RPM, hood, flight time -- is solved from this pose. */
        public final Pose predictedRobotPose;
        /** Robot pose predicted ahead by the turret servo delay (now + turret latency), so the lagging
         *  servo tracks the target in real time and is on-target at the ball's release. Aim the turret
         *  FROM this pose; everything else uses {@link #predictedRobotPose}. */
        public final Pose predictedTurretPose;
        public final Pose virtualGoal;
        public final double distancePoseUnits;
        public final double targetRpm;
        public final double hoodBaselineDegFromVertical;
        public final double flightTimeSec;
        public final boolean possible;

        public final double robotAxFieldPosePerSec2;
        public final double robotAyFieldPosePerSec2;
        public final double robotAlphaRadPerSec2;
        /** Lag- and stop-compensated velocity the lead was actually built from (field in/s). Compare
         *  to the raw measured velocity: near a braked stop this should fall to ~0 while the raw
         *  reading still lags. Use it to tune {@code VELOCITY_LATENCY_SEC}. */
        public final double launchVxFieldPosePerSec;
        public final double launchVyFieldPosePerSec;
        public ShotCommand(Pose predictedRobotPose,
                           Pose predictedTurretPose,
                           Pose virtualGoal,
                           double distancePoseUnits,
                           double targetRpm,
                           double hoodBaselineDegFromVertical,
                           double flightTimeSec,
                           boolean possible,
                           double robotAxFieldPosePerSec2,
                           double robotAyFieldPosePerSec2,
                           double robotAlphaRadPerSec2,
                           double launchVxFieldPosePerSec,
                           double launchVyFieldPosePerSec) {
            this.predictedRobotPose = predictedRobotPose;
            this.predictedTurretPose = predictedTurretPose;
            this.virtualGoal = virtualGoal;
            this.distancePoseUnits = distancePoseUnits;
            this.targetRpm = targetRpm;
            this.hoodBaselineDegFromVertical = hoodBaselineDegFromVertical;
            this.flightTimeSec = flightTimeSec;
            this.possible = possible;
            this.robotAxFieldPosePerSec2 = robotAxFieldPosePerSec2;
            this.robotAyFieldPosePerSec2 = robotAyFieldPosePerSec2;
            this.robotAlphaRadPerSec2 = robotAlphaRadPerSec2;
            this.launchVxFieldPosePerSec = launchVxFieldPosePerSec;
            this.launchVyFieldPosePerSec = launchVyFieldPosePerSec;
        }
    }

    /** Result of integrating the motion model over a latency horizon: the predicted pose components
     *  plus the predicted field velocity AT that horizon (used as the launch-instant velocity). */
    private static final class Predicted {
        final double x, y, heading, vx, vy;
        Predicted(double x, double y, double heading, double vx, double vy) {
            this.x = x; this.y = y; this.heading = heading; this.vx = vx; this.vy = vy;
        }
    }

    private static class AccelSample {
        public final double ax;
        public final double ay;
        public final double alpha;

        public AccelSample(double ax, double ay, double alpha) {
            this.ax = ax;
            this.ay = ay;
            this.alpha = alpha;
        }
    }

    public static double POSE_UNITS_TO_METERS = 0.0254;
    public static double EXIT_VEL_M_PER_RPM_CLOSE = 0.00365;       // NEAR coefficient (your tuned value)
    public static double EXIT_VEL_M_PER_RPM_FAR = 0.00357;   // FAR coefficient (tune this)
    public static double EXIT_VEL_SPLIT_THRESHOLD_M = 2.97;   // distance (m) at/above which FAR is used

    // Master gate for the whole latency-compensation / pose-prediction path. When off, the solve
    // describes the robot exactly where it is now (legacy behavior). When on, the latency model below
    // predicts the robot forward so each actuator tracks where the robot WILL be at release.
    public static boolean ENABLE_FUTURE_POSE_PREDICTION = false;

    // ===== Explicit latency model (replaces the old single lumped SHOT_PREDICTION_SEC) =====
    // These two horizons are INDEPENDENT, not additive: each actuator is led by its own delay.
    //
    // RELEASE_LATENCY_SEC: time from the "run transfer" hardware call to the ball actually leaving the
    // shooter. The ball is already sitting against the flywheel, so this is physically SMALL. It sets
    // the horizon for the shot geometry (distance -> RPM/hood) and for the SOTM lead's launch velocity
    // -- the ball inherits the robot velocity at this (near-current) instant.
    public static double RELEASE_LATENCY_SEC = 0.03;
    // TURRET_LATENCY_SEC: the turret servo's response delay (transport + slew). The turret aim is led
    // by exactly this, so the lagging servo TRACKS THE TARGET IN REAL TIME: the angle commanded now is
    // the one the servo will physically reach TURRET_LATENCY_SEC later, so its actual angle at any
    // instant equals the angle wanted at that instant (and is therefore correct at the ball's release).
    // Normally MUCH larger than RELEASE_LATENCY_SEC. Tune to the measured servo delay; only affects the
    // pose the turret aims FROM -- never distance/RPM/hood/lead.
    public static double TURRET_LATENCY_SEC = 0.135;
    // VELOCITY_LATENCY_SEC: lag of the VELOCITY estimate itself (Pedro's filtered velocity is a delayed
    // derivative; odometry POSITION is fresh, velocity is not). The SOTM lead is launch_velocity *
    // flight_time, so a stale velocity is what breaks decel/reverse shots: when you brake to a near-stop
    // and fire, the lagging reading still says "moving backward" and SOTM applies a phantom backward
    // lead. We undo that by projecting the measured velocity forward by this lag (using the estimated
    // acceleration) BEFORE forming the lead, with the same stop-clamp -- so near a stop the lead
    // correctly collapses to ~0. At constant speed this is a no-op (velocity isn't changing). Tune to
    // the measured velocity-estimate delay; only affects the lead's launch velocity.
    public static double VELOCITY_LATENCY_SEC = 0.06;

    // Acceleration IS used for the forward prediction -- over the ~0.15s turret servo delay (≈4 loops)
    // the target moves enough that ignoring accel leaves the turret mis-led during a decel. The catch
    // is the SOURCE: at this robot's ~40ms (25Hz) loop a multi-sample regression spans ~0.2s and reports
    // STALE, pre-brake acceleration, which is what made the turret over-lead ("faster velocity, shoots
    // to the side"). Instead we use a fresh ONE-STEP finite difference (this loop's velocity minus last
    // loop's) with a light EMA: ~1 loop of lag instead of ~5, and the slow turret servo filters the
    // residual noise. Set false for a pure constant-velocity fallback.
    public static boolean USE_ACCEL_PREDICTION = true;
    // EMA weight on the freshest one-step accel (0..1). Higher = fresher but noisier; lower = smoother
    // but laggier. Raised toward fresher so the accel (and thus the velocity-lag correction + turret
    // lead) reacts quickly to a hard brake; the slow turret servo filters the extra noise. 1.0 = raw
    // one-step diff (no smoothing).
    public static double ACCEL_SMOOTHING_ALPHA = 0.3;
    public static double MIN_SAMPLE_SPACING_SEC = 0.008;
    public static double MAX_LINEAR_ACCEL_POSE_PER_SEC2 = 200.0;
    public static double MAX_ANGULAR_ACCEL_RAD_PER_SEC2 = 20.0;

    public static boolean MOVE_SHOT_ENABLED = false;
    public static int MOVE_SHOT_ITERS = 2;
    public static double MOVE_SHOT_MAX_LEAD_POSE_UNITS = 100.0;

    public static double MAX_RPM = 2800.0;
    public static double MIN_HOOD_ANGLE_DEG = 30;
    public static double MAX_HOOD_ANGLE_DEG = 55;

    public static double FAR_SHOT_THRESHOLD_M = 1.7;
    public static double FAR_SHOT_MIN_HOOD_DEG = 30;
    public static double FAR_SHOT_MAX_HOOD_DEG = 55;

    public static int RPM_SMOOTHING_WINDOW_SAMPLES = 8;

    public static double GOAL_HEIGHT_M = 1.2615; // 1.2575
    // Treat the goal as a vertical opening, not a single point: a shot counts if it crosses the
    // goal plane anywhere within GOAL_HEIGHT_M +/- GOAL_ACCEPT_BAND_M. Widen for more RPM leniency.
    public static double GOAL_ACCEPT_BAND_M = 0.06; // 0.06
    // How many candidate crossing heights to test across the opening band, from the center outward
    // to both edges. 1 = center only (original behavior); higher = more intermediate points are
    // checked, which lets the solver find a valid hood angle in more borderline geometries instead
    // of giving up when only the exact-center angle is tried. Odd values keep the sampling
    // symmetric about the center.
    public static int GOAL_HEIGHT_SAMPLES = 7;
    public static double GOAL_LIP_HEIGHT_M = 1.0;
    public static double GOAL_DEPTH_M = 1 * 0.0254;
    public static double LAUNCHER_HEIGHT_ACTUAL_M = 0.38;
    public static double GRAVITY = 9.80665;

    private final double[] distances = {
            50, 53, 57.777, 62.5, 67.5, 72.5, 99.2, 121.59989031660349, 133.5, 145.2, 150, 161.0177438683991
    };

    private final double[] velocities = {
            1240, 1300, 1440, 1540, 1560, 1580, 1680, 1725, 1900, 1950, 1975, 2025 // -75 for far far - last 2, -60 for far
    };

    /** Exit-velocity-per-RPM coefficient, switched by horizontal shot distance (meters). */
    private static double exitVelPerRpm(double xMeters) {
        return xMeters > EXIT_VEL_SPLIT_THRESHOLD_M ? EXIT_VEL_M_PER_RPM_FAR : EXIT_VEL_M_PER_RPM_CLOSE;
    }

    private final InterpLUT velocityLut = new InterpLUT();

    private final ElapsedTime motionTimer = new ElapsedTime();
    private final ArrayDeque<Double> rpmHistory = new ArrayDeque<>();
    // One-step accel estimator state: last loop's velocity + time, and the EMA-smoothed accel.
    private boolean haveAccelPrev = false;
    private double prevAccelTimeSec = 0.0;
    private double prevVxForAccel = 0.0, prevVyForAccel = 0.0, prevOmegaForAccel = 0.0;
    private double axEma = 0.0, ayEma = 0.0, alphaEma = 0.0;
    public static JoinedTelemetry tel;

    public ShotPlanner() {
        for (int i = 0; i < distances.length; i++) {
            velocityLut.add(distances[i], velocities[i]);
        }
        velocityLut.createLUT();
        motionTimer.reset();
        MOVE_SHOT_ENABLED = false;
        ENABLE_FUTURE_POSE_PREDICTION = false;
    }

    public static void enableSOTM(){
        MOVE_SHOT_ENABLED = true;
    }

    public static void disableSOTM(){
        MOVE_SHOT_ENABLED = false;
    }

    public static void enableFPP(){
        ENABLE_FUTURE_POSE_PREDICTION = true;
    }

    public static void disableFPP() {
        ENABLE_FUTURE_POSE_PREDICTION = false;
    }

    private double finiteOrZero(double x) {
        return Double.isFinite(x) ? x : 0.0;
    }

    private double wrapAngleRad(double angleRad) {
        if (!Double.isFinite(angleRad)) return 0.0;
        return Math.atan2(Math.sin(angleRad), Math.cos(angleRad));
    }

    private double clampDist(double distPoseUnits) {
        return MathUtils.clamp(
                distPoseUnits,
                distances[0] * 1.01,
                distances[distances.length - 1] * 0.99
        );
    }

    private double hoodDegToThetaRadFromHorizontal(double hoodDegFromVertical) {
        return Math.toRadians(90.0 - hoodDegFromVertical);
    }

    private double addRpmSample(double rpm) {
        rpm = finiteOrZero(rpm);

        int maxSamples = Math.max(1, RPM_SMOOTHING_WINDOW_SAMPLES);
        rpmHistory.addLast(rpm);
        while (rpmHistory.size() > maxSamples) {
            rpmHistory.pollFirst();
        }

        double sum = 0.0;
        for (double sample : rpmHistory) {
            sum += sample;
        }
        return sum / rpmHistory.size();
    }

    /**
     * Update and return the EMA-smoothed acceleration from a fresh ONE-STEP finite difference of the
     * field velocity (this loop minus last). Chosen over a multi-sample regression because at ~40ms
     * loops the regression lags ~0.2s and reports stale pre-brake accel; a one-step diff lags ~1 loop,
     * which is what the turret's 0.15s prediction needs to track a quick decel. The EMA (and the slow
     * servo) tame the inherent noise. Always maintained so the estimate is warm and shown in telemetry.
     */
    private AccelSample updateAcceleration(double vx, double vy, double omega) {
        vx = finiteOrZero(vx);
        vy = finiteOrZero(vy);
        omega = finiteOrZero(omega);

        double nowSec = motionTimer.seconds();
        if (!Double.isFinite(nowSec)) return new AccelSample(axEma, ayEma, alphaEma);

        if (!haveAccelPrev) {
            haveAccelPrev = true;
            prevAccelTimeSec = nowSec;
            prevVxForAccel = vx; prevVyForAccel = vy; prevOmegaForAccel = omega;
            return new AccelSample(0.0, 0.0, 0.0);
        }

        double dt = nowSec - prevAccelTimeSec;
        if (dt < MIN_SAMPLE_SPACING_SEC) {
            // Too soon to differentiate reliably; hold the current estimate without advancing prev.
            return new AccelSample(axEma, ayEma, alphaEma);
        }

        double rawAx = (vx - prevVxForAccel) / dt;
        double rawAy = (vy - prevVyForAccel) / dt;
        double rawAlpha = (omega - prevOmegaForAccel) / dt;

        double linClamp = Math.abs(MAX_LINEAR_ACCEL_POSE_PER_SEC2);
        double angClamp = Math.abs(MAX_ANGULAR_ACCEL_RAD_PER_SEC2);
        rawAx = MathUtils.clamp(finiteOrZero(rawAx), -linClamp, linClamp);
        rawAy = MathUtils.clamp(finiteOrZero(rawAy), -linClamp, linClamp);
        rawAlpha = MathUtils.clamp(finiteOrZero(rawAlpha), -angClamp, angClamp);

        double a = MathUtils.clamp(ACCEL_SMOOTHING_ALPHA, 0.0, 1.0);
        axEma += a * (rawAx - axEma);
        ayEma += a * (rawAy - ayEma);
        alphaEma += a * (rawAlpha - alphaEma);

        prevAccelTimeSec = nowSec;
        prevVxForAccel = vx; prevVyForAccel = vy; prevOmegaForAccel = omega;

        return new AccelSample(axEma, ayEma, alphaEma);
    }

    public void resetMotionHistory() {
        haveAccelPrev = false;
        axEma = ayEma = alphaEma = 0.0;
        motionTimer.reset();
    }

    private double effectiveHorizon(double configuredSec) {
        if (!ENABLE_FUTURE_POSE_PREDICTION) return 0.0;
        return Math.max(0.0, finiteOrZero(configuredSec));
    }

    /**
     * Integrate one axis over [0, t] under constant accel {@code a}, but never let braking pull the
     * velocity through zero into a fake reversal: a decelerating robot coasts to a stop and STAYS
     * there, it does not spring backward. This is the key fix for the decel-to-stop shot -- a plain
     * x + v*t + 1/2*a*t^2 extrapolation (with a lagging, over-estimated braking accel) would predict
     * the robot overshooting the stop point or even reversing exactly at the instant we fire.
     *
     * @return {@code [predictedPosition, predictedVelocityAtT]}; velocity is 0 once stopped.
     */
    private static double[] integrateAxisStopClamped(double pos0, double v, double a, double t) {
        if (t <= 0.0) return new double[]{pos0, v};

        double te = t;                 // effective integration time
        boolean braking = v != 0.0 && Math.signum(a) == -Math.signum(v);
        if (braking) {
            double tStop = -v / a;     // > 0: when this axis reaches zero velocity
            if (tStop < t) te = tStop; // stop early, then hold
        }

        double pos = pos0 + v * te + 0.5 * a * te * te;
        double vEnd = (te < t) ? 0.0 : v + a * t; // after a within-horizon stop, velocity stays 0
        return new double[]{pos, vEnd};
    }

    /** Shift a laggy measured velocity to its true current value: {@code v + a * lag}. Won't flip the
     *  sign -- a brake decelerates toward rest, it doesn't spontaneously reverse -- so if the correction
     *  would cross zero it clamps to 0 (the robot has effectively stopped). */
    private double compensateVel(double vMeasured, double a, double lagSec) {
        double vc = vMeasured + a * lagSec;
        if (vMeasured != 0.0 && Math.signum(vc) != Math.signum(vMeasured)) return 0.0;
        return vc;
    }

    /** Predict the robot pose AND launch-instant velocity over {@code horizonSec}, with each axis
     *  stop-clamped (see {@link #integrateAxisStopClamped}) so braking can't fabricate a reversal. */
    private Predicted predictWithStop(Pose robotPose,
                                      double vx, double vy, double omega,
                                      double ax, double ay, double alpha,
                                      double horizonSec) {
        double t = Math.max(0.0, finiteOrZero(horizonSec));

        double[] sx = integrateAxisStopClamped(robotPose.getX(), vx, ax, t);
        double[] sy = integrateAxisStopClamped(robotPose.getY(), vy, ay, t);
        double[] sh = integrateAxisStopClamped(robotPose.getHeading(), omega, alpha, t);

        return new Predicted(sx[0], sy[0], wrapAngleRad(sh[0]), sx[1], sy[1]);
    }

    /** Trajectory height above the launcher at horizontal distance {@code d}, for launch angle theta. */
    private static double trajHeightAboveLauncher(double theta, double d, double v2, double g) {
        double cos = Math.cos(theta);
        if (Math.abs(cos) < 1e-6) return Double.NEGATIVE_INFINITY;
        return d * Math.tan(theta) - (g * d * d) / (2.0 * v2 * cos * cos);
    }

    /**
     * Aim at the center of the opening, then clamp that ideal angle into the hood's mechanical
     * range. The shot is accepted as long as the (possibly clamped) trajectory still crosses the
     * goal plane inside the opening band [minDh, maxDh] and clears the front lip. Returns the hood
     * angle (deg from vertical) if valid, else null.
     */
    private static Double tryArc(double thetaCenter,
                                 double thetaFlatMech, double thetaSteepMech,
                                 double x, double xLip, double v2, double g,
                                 double minDh, double maxDh, String label) {
        if (Double.isNaN(thetaCenter)) return null;

        double theta = MathUtils.clamp(thetaCenter, thetaFlatMech, thetaSteepMech);

        double h = trajHeightAboveLauncher(theta, x, v2, g);
        if (h < minDh || h > maxDh) return null;

        if (xLip > 0.0) {
            double hLip = trajHeightAboveLauncher(theta, xLip, v2, g);
            if (hLip + LAUNCHER_HEIGHT_ACTUAL_M < GOAL_LIP_HEIGHT_M) return null;
        }

        tel.addLine(label);
        return 90.0 - Math.toDegrees(theta);
    }

    /**
     * Candidate crossing heights (above the launcher) to try, ordered center-first and then
     * expanding outward toward both edges of the opening band. Testing several heights — not just
     * the exact center — lets the solver accept a shot whenever ANY height within the opening is
     * reachable with a mechanically-valid hood angle, instead of giving up when the center-aimed
     * angle alone clamps out of range. The band edges (minDh / maxDh) are the outermost samples.
     */
    private static double[] sampledHeights(double centerDh, double minDh, double maxDh, int samples) {
        if (samples <= 1) return new double[]{centerDh};

        double[] out = new double[samples];
        out[0] = centerDh;

        int pairs = samples / 2;             // how many heights to place on each side of center
        double upSpan = maxDh - centerDh;
        double downSpan = centerDh - minDh;

        int idx = 1;
        for (int k = 1; k <= pairs && idx < samples; k++) {
            double frac = (double) k / pairs; // reaches 1.0 (the band edge) at k == pairs
            if (idx < samples) out[idx++] = centerDh + upSpan * frac;
            if (idx < samples) out[idx++] = centerDh - downSpan * frac;
        }
        return out;
    }

    public static double solveHoodDeg(double distPoseUnits, double rpm) {
        double x = distPoseUnits * POSE_UNITS_TO_METERS;
        double v = Math.abs(rpm) * exitVelPerRpm(x);
        double xLip = x - GOAL_DEPTH_M;

        if (x <= 1e-6 || v <= 1e-6) return Double.NaN;

        double g = GRAVITY;
        double v2 = v * v;
        double v4 = v2 * v2;

        // Goal opening as a vertical band (heights above the launcher) instead of a single point.
        double centerDh = GOAL_HEIGHT_M - LAUNCHER_HEIGHT_ACTUAL_M;
        double minDh = (GOAL_HEIGHT_M - GOAL_ACCEPT_BAND_M) - LAUNCHER_HEIGHT_ACTUAL_M;
        double maxDh = (GOAL_HEIGHT_M + GOAL_ACCEPT_BAND_M) - LAUNCHER_HEIGHT_ACTUAL_M;

        boolean isFarShot = x > FAR_SHOT_THRESHOLD_M;
        double minHood = isFarShot ? FAR_SHOT_MIN_HOOD_DEG : MIN_HOOD_ANGLE_DEG;
        double maxHood = isFarShot ? FAR_SHOT_MAX_HOOD_DEG : MAX_HOOD_ANGLE_DEG;

        // Hood mechanical limits expressed as launch angle from horizontal.
        double thetaFlatMech  = Math.toRadians(90.0 - maxHood); // flattest the hood allows
        double thetaSteepMech = Math.toRadians(90.0 - minHood); // steepest the hood allows

        // Walk candidate crossing heights from the center outward to both edges of the opening. For
        // each, solve the two launch angles (low / high arc) that pass through it exactly, then let
        // tryArc clamp into the hood range and confirm the shot still crosses inside the band and
        // clears the lip. The first valid candidate wins, and the center is tried first, so the
        // most forgiving shot is always preferred. anyReachable distinguishes "no height in the
        // opening is physically reachable at this speed/distance" from "reachable but no valid hood
        // angle within mechanical limits."
        boolean anyReachable = false;
        for (double targetDh : sampledHeights(centerDh, minDh, maxDh, Math.max(1, GOAL_HEIGHT_SAMPLES))) {
            double disc = v4 - g * (g * x * x + 2.0 * targetDh * v2);
            if (disc < 0.0) continue; // this height isn't reachable at this speed + distance
            anyReachable = true;

            double sqrtDisc = Math.sqrt(disc);
            double thetaLo = Math.atan((v2 - sqrtDisc) / (g * x));
            double thetaHi = Math.atan((v2 + sqrtDisc) / (g * x));

            Double low = tryArc(thetaLo, thetaFlatMech, thetaSteepMech, x, xLip, v2, g, minDh, maxDh, "[AUTO AIM] LOW ARC");
            if (low != null) return low;

            Double high = tryArc(thetaHi, thetaFlatMech, thetaSteepMech, x, xLip, v2, g, minDh, maxDh, "[AUTO AIM] HIGH ARC");
            if (high != null) return high;
        }

        tel.addLine(anyReachable ? "[AUTO AIM] IMPOSSIBLE" : "[AUTO AIM] IMPOSSIBLE (unreachable)");
        return Double.NaN;
    }

    private double estimateFlightTimeSec(double distPoseUnits, double rpm, double hoodDegFromVertical) {
        double xMeters = distPoseUnits * POSE_UNITS_TO_METERS;
        double theta = hoodDegToThetaRadFromHorizontal(hoodDegFromVertical);

        double vExit = Math.abs(rpm) * exitVelPerRpm(xMeters);
        double vHoriz = vExit * Math.cos(theta);

        if (vHoriz < 1e-3) return Double.NaN;
        return xMeters / vHoriz;
    }

    private Pose computeVirtualGoalIter(Pose robotPoseForPlanning,
                                        Pose realGoalPose,
                                        double launchVxFieldPosePerSec,
                                        double launchVyFieldPosePerSec,
                                        double rpm) {
        if (!MOVE_SHOT_ENABLED) return realGoalPose;

        Pose virtualGoal = realGoalPose;

        for (int i = 0; i < MOVE_SHOT_ITERS; i++) {
            double dx = virtualGoal.getX() - robotPoseForPlanning.getX();
            double dy = virtualGoal.getY() - robotPoseForPlanning.getY();
            double distPose = Math.hypot(dx, dy);
            double distClamped = clampDist(distPose);

            // Solve at the flywheel's CURRENT (smoothed measured) RPM: the wheel changes speed slowly
            // and we fire immediately, so the ball leaves at whatever speed the wheel has now -- that
            // is the speed the lead's flight time must be based on.
            double hood = solveHoodDeg(distClamped, rpm);
            if (!Double.isFinite(hood)) return virtualGoal;

            double t = estimateFlightTimeSec(distClamped, rpm, hood);
            if (!Double.isFinite(t)) return virtualGoal;

            double leadX = launchVxFieldPosePerSec * t;
            double leadY = launchVyFieldPosePerSec * t;

            double leadMag = Math.hypot(leadX, leadY);
            if (leadMag > MOVE_SHOT_MAX_LEAD_POSE_UNITS && leadMag > 1e-6) {
                double s = MOVE_SHOT_MAX_LEAD_POSE_UNITS / leadMag;
                leadX *= s;
                leadY *= s;
            }

            virtualGoal = new Pose(
                    realGoalPose.getX() - leadX,
                    realGoalPose.getY() - leadY,
                    realGoalPose.getHeading()
            );
        }

        return virtualGoal;
    }

    public ShotCommand plan(Pose robotPose,
                            double robotVxFieldPosePerSec,
                            double robotVyFieldPosePerSec,
                            double robotOmegaRadPerSec,
                            Pose realGoalPose, double rpm) {

        double smoothedRpm = addRpmSample(rpm);

        double vx = finiteOrZero(robotVxFieldPosePerSec);
        double vy = finiteOrZero(robotVyFieldPosePerSec);
        double omega = finiteOrZero(robotOmegaRadPerSec);

        // Fresh one-step EMA accel (always maintained; also surfaced in telemetry). Gate whether the
        // prediction actually uses it -- off => clean constant-velocity fallback.
        AccelSample accelEst = updateAcceleration(vx, vy, omega);
        AccelSample accel = USE_ACCEL_PREDICTION ? accelEst : new AccelSample(0.0, 0.0, 0.0);

        // Compensate the velocity-estimate lag BEFORE any prediction. The measured velocity is a delayed,
        // filtered derivative, so during a hard brake it reads too fast -- "the robot thinks it's going
        // faster than it is" -- and the turret, projecting that over its 0.15s servo horizon, over-leads.
        // Shift the measured velocity to its true current value using the fresh accel, and feed THAT into
        // every prediction (turret pose, shot pose, and lead). This is the fix for braking-shot misses.
        double velLag = ENABLE_FUTURE_POSE_PREDICTION ? Math.max(0.0, VELOCITY_LATENCY_SEC) : 0.0;
        double vxc = compensateVel(vx, accel.ax, velLag);
        double vyc = compensateVel(vy, accel.ay, velLag);
        double omegac = compensateVel(omega, accel.alpha, velLag);

        // Independent actuator horizons. Shot geometry: small release latency. Turret: its servo delay,
        // so the slow servo tracks the (now lag-corrected) target in real time and is right at release.
        double shotHorizon = effectiveHorizon(RELEASE_LATENCY_SEC);
        double turretHorizon = effectiveHorizon(TURRET_LATENCY_SEC);

        Predicted shotPred = predictWithStop(
                robotPose, vxc, vyc, omegac, accel.ax, accel.ay, accel.alpha, shotHorizon);
        Predicted turretPred = predictWithStop(
                robotPose, vxc, vyc, omegac, accel.ax, accel.ay, accel.alpha, turretHorizon);

        Pose predictedPose = new Pose(shotPred.x, shotPred.y, shotPred.heading);
        Pose predictedTurretPose = new Pose(turretPred.x, turretPred.y, turretPred.heading);

        // The ball inherits the robot's true velocity at release: the lag-corrected velocity projected
        // to the release instant (what shotPred already carries), stop-clamped so the lead collapses to
        // ~0 at a stop instead of leaving a phantom backward lead.
        double predictedLaunchVx = shotPred.vx;
        double predictedLaunchVy = shotPred.vy;

        Pose virtualGoal = computeVirtualGoalIter(
                predictedPose,
                realGoalPose,
                predictedLaunchVx,
                predictedLaunchVy,
                smoothedRpm
        );

        double dx = virtualGoal.getX() - predictedPose.getX();
        double dy = virtualGoal.getY() - predictedPose.getY();
        double distPose = Math.hypot(dx, dy);
        double distClamped = clampDist(distPose);

        // targetRPM is the LUT setpoint the (slow) flywheel is driven toward, but the hood and flight
        // time are solved at the CURRENT measured RPM so the shot is valid at whatever speed the wheel
        // actually has right now -- letting it fire mid-spin-up instead of waiting for the setpoint.
        double targetRPM = Range.clip(velocityLut.get(distClamped), 0.0, MAX_RPM);
        double hood = solveHoodDeg(distClamped, smoothedRpm);
        boolean possible = Double.isFinite(hood);

        double flightTimeSec = possible ? estimateFlightTimeSec(distClamped, smoothedRpm, hood) : Double.NaN;

        return new ShotCommand(
                predictedPose,
                predictedTurretPose,
                virtualGoal,
                distClamped,
                targetRPM,
                hood,
                flightTimeSec,
                possible,
                accel.ax,
                accel.ay,
                accel.alpha,
                predictedLaunchVx,
                predictedLaunchVy
        );
    }

    public double getLutRpm(double distPoseUnits) {
        return Range.clip(velocityLut.get(clampDist(distPoseUnits)), 0.0, MAX_RPM);
    }

    public double getPhysicsHoodDeg(double distPoseUnits) {
        return solveHoodDeg(clampDist(distPoseUnits), getLutRpm(distPoseUnits));
    }
}