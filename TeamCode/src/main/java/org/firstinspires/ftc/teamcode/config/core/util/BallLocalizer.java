package org.firstinspires.ftc.teamcode.config.core.util;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.teamcode.config.core.util.robothelper.Artifact;

import java.util.ArrayList;
import java.util.List;

/**
 * Turns the Limelight neural-detector output into tracked balls in field (Pedro)
 * coordinates, and estimates each ball's velocity.
 *
 * <p><b>Why projection happens here and not on the Limelight:</b> the detector only knows
 * camera angles. Converting those to a field position needs the robot's odometry pose, and
 * estimating ball velocity needs the robot's own motion removed — both of which live on
 * the robot. By projecting every frame to the field first and then differencing, the
 * robot's translation/rotation cancels out automatically.
 *
 * <p><b>Geometry model:</b> a pinhole camera mounted on the robot, pointing forward along
 * the robot heading (plus an optional yaw offset), tilted down from horizontal by
 * {@link #CAMERA_PITCH_DEG_DOWN}. Each detection's angles define a ray; we intersect that
 * ray with the horizontal plane at the ball's center height ({@link #BALL_RADIUS_IN}) to
 * get the ball's field XY.
 *
 * <p>Field convention matches the rest of the code: X/Y in inches, heading in radians with
 * forward = (cos h, sin h). All camera mount numbers below are placeholders — measure them
 * on the real robot (they are {@code @Configurable} so they can be tuned live in Panels).
 *
 * <h3>Usage</h3>
 * <pre>{@code
 * BallLocalizer localizer = new BallLocalizer();
 * // each loop, after switching the Limelight to the neural detector pipeline:
 * localizer.update(llResult.getDetectorResults(), follower.getPose(), runtimeNow);
 * List<Ball> balls = localizer.getBalls();   // field coords + velocity
 * }</pre>
 */
@Configurable
public class BallLocalizer {

    // ---- Camera mount geometry (MEASURE THESE on the real robot) ----
    /** Camera lens height above the floor, inches. */
    public static double CAMERA_HEIGHT_IN = 8.56;
    /** Downward tilt of the optical axis from horizontal, degrees. */
    public static double CAMERA_PITCH_DEG_DOWN = 9.193077;
    /** Yaw of the camera relative to robot forward, degrees (+ = aimed to robot's right). */
    public static double CAMERA_YAW_OFFSET_DEG = 0.0;
    /** Camera position ahead of the robot's rotation center, inches (+ = forward). */
    public static double CAMERA_FORWARD_OFFSET_IN = 8.43;
    /** Camera position to the left of the robot's rotation center, inches (+ = left). */
    public static double CAMERA_LEFT_OFFSET_IN = 0.0;
    /** Physical ball radius, inches — the ball's center sits this high off the floor. */
    public static double BALL_RADIUS_IN = 2.45;

    // ---- Detector classes ----
    /** Detector class names are matched case-insensitively by "contains" against these. */
    public static String PURPLE_CLASS = "purple";
    public static String GREEN_CLASS = "green";
    /** Ignore detections below this confidence (0..1). */
    public static double MIN_CONFIDENCE = 0.5;

    // ---- Tracking / velocity ----
    /** A detection within this distance of an existing track (same color) updates it. */
    public static double MATCH_RADIUS_IN = 8.0;
    /** Position EMA smoothing in [0,1]; higher = snappier, lower = steadier. */
    public static double POSITION_ALPHA = 0.6;
    /** Velocity is least-squares fit over measurements within this time window, seconds.
     *  Longer = smoother but laggier. This is the main knob for spiky velocity. */
    public static double VELOCITY_WINDOW_SEC = 0.35;
    /** Hard cap on samples kept for the velocity fit. */
    public static int VELOCITY_MAX_SAMPLES = 15;
    /** Drop tracks not seen for this long, seconds. */
    public static double STALE_SEC = 0.4;
    /** Clamp fitted ball speed to at most this, inches/sec. */
    public static double MAX_BALL_SPEED_IN_S = 120.0;
    /** Reject detections projected outside the field (with this margin), inches. */
    public static double FIELD_SIZE_IN = 141.5;
    public static double FIELD_MARGIN_IN = 2.0;

    private final List<Ball> tracks = new ArrayList<>();
    private int nextId = 0;

    /**
     * Feed one frame of detector output, i.e. the list from
     * {@code LLResult.getDetectorResults()}. Safe to call with {@code null} / empty lists.
     */
    public void update(List<LLResultTypes.DetectorResult> detections, Pose robotPose, double nowSec) {
        if (detections != null && robotPose != null) {
            for (LLResultTypes.DetectorResult d : detections) {
                if (d.getConfidence() < MIN_CONFIDENCE) continue;

                Artifact color = colorFromClassName(d.getClassName());
                if (color == Artifact.NONE) continue;

                // Limelight tx/ty: + = right / + = up, which matches projectToField.
                Pose field = projectToField(d.getTargetXDegrees(), d.getTargetYDegrees(), robotPose);
                if (field == null) continue;
                if (outOfField(field.getX(), field.getY())) continue;

                associate(field, color, d.getConfidence(), nowSec);
            }
        }
        pruneStale(nowSec);
    }

    /** Intersect the detection ray with the plane z = ball-center height; null if it can't hit. */
    private Pose projectToField(double angleXDeg, double angleYDeg, Pose robotPose) {
        double az = Math.toRadians(angleXDeg);  // + = right of camera axis
        double el = Math.toRadians(angleYDeg);  // + = above camera axis
        double h = robotPose.getHeading() - Math.toRadians(CAMERA_YAW_OFFSET_DEG);
        double pitch = Math.toRadians(CAMERA_PITCH_DEG_DOWN);

        // Camera basis expressed in field coordinates (X east, Y north, Z up).
        // Optical axis: along heading, tilted down by pitch.
        double[] forward = {Math.cos(h) * Math.cos(pitch), Math.sin(h) * Math.cos(pitch), -Math.sin(pitch)};
        // Image +x (right of the camera) is horizontal, 90 deg clockwise from heading.
        double[] right = {Math.sin(h), -Math.cos(h), 0.0};
        // Image +y (up) completes the right-handed set (x=right, y=up, z=forward): up = right x forward.
        double[] up = cross(right, forward);

        // Ray direction for a detection at (az, el): forward + tan(az)*right + tan(el)*up.
        double tx = Math.tan(az);
        double ty = Math.tan(el);
        double dx = forward[0] + tx * right[0] + ty * up[0];
        double dy = forward[1] + tx * right[1] + ty * up[1];
        double dz = forward[2] + tx * right[2] + ty * up[2];

        // Camera origin in the field.
        double camX = robotPose.getX() + CAMERA_FORWARD_OFFSET_IN * Math.cos(h) - CAMERA_LEFT_OFFSET_IN * Math.sin(h);
        double camY = robotPose.getY() + CAMERA_FORWARD_OFFSET_IN * Math.sin(h) + CAMERA_LEFT_OFFSET_IN * Math.cos(h);
        double camZ = CAMERA_HEIGHT_IN;

        // Solve camZ + t*dz = BALL_RADIUS_IN. Need the ray heading downward to a point below us.
        if (dz >= -1e-6) return null;
        double t = (BALL_RADIUS_IN - camZ) / dz;
        if (t <= 0) return null;

        return new Pose(camX + t * dx, camY + t * dy, 0);
    }

    /**
     * The camera lens position in field coordinates for a given robot pose (heading stored is
     * the camera's, ignore Z). Useful for reading off camera-to-ball distance when calibrating.
     */
    public Pose cameraOrigin(Pose robotPose) {
        double h = robotPose.getHeading() - Math.toRadians(CAMERA_YAW_OFFSET_DEG);
        double camX = robotPose.getX() + CAMERA_FORWARD_OFFSET_IN * Math.cos(h) - CAMERA_LEFT_OFFSET_IN * Math.sin(h);
        double camY = robotPose.getY() + CAMERA_FORWARD_OFFSET_IN * Math.sin(h) + CAMERA_LEFT_OFFSET_IN * Math.cos(h);
        return new Pose(camX, camY, h);
    }

    /** Match to the nearest same-color track within MATCH_RADIUS_IN, else start a new track. */
    private void associate(Pose measured, Artifact color, double confidence, double nowSec) {
        Ball best = null;
        double bestDistSq = MATCH_RADIUS_IN * MATCH_RADIUS_IN;
        for (Ball b : tracks) {
            if (b.color != color) continue;
            double d2 = b.position.distSquared(measured);
            if (d2 <= bestDistSq) {
                bestDistSq = d2;
                best = b;
            }
        }

        if (best == null) {
            Ball nb = new Ball(nextId++, color, measured, nowSec, confidence);
            nb.pushSample(nowSec, measured.getX(), measured.getY());
            tracks.add(nb);
            return;
        }

        best.position = best.position.plus(measured.minus(best.position).times(POSITION_ALPHA));
        best.confidence = confidence;
        best.lastSeenSec = nowSec;

        // Velocity from a least-squares line fit over recent raw measurements (smooth, not
        // a noisy single-frame difference).
        best.pushSample(nowSec, measured.getX(), measured.getY());
        best.velocity = fitVelocity(best, nowSec);
    }

    /** Least-squares slope of x-vs-t and y-vs-t over the recent sample window. */
    private Pose fitVelocity(Ball b, double nowSec) {
        while (b.history.size() > VELOCITY_MAX_SAMPLES) b.history.pollFirst();
        while (b.history.size() > 2 && nowSec - b.history.peekFirst()[0] > VELOCITY_WINDOW_SEC) {
            b.history.pollFirst();
        }

        int n = b.history.size();
        if (n < 2) return new Pose(0, 0, 0);

        double t0 = b.history.peekFirst()[0];
        double sumT = 0, sumX = 0, sumY = 0;
        for (double[] s : b.history) { sumT += s[0] - t0; sumX += s[1]; sumY += s[2]; }
        double meanT = sumT / n, meanX = sumX / n, meanY = sumY / n;

        double sxx = 0, sxX = 0, sxY = 0;
        for (double[] s : b.history) {
            double dt = (s[0] - t0) - meanT;
            sxx += dt * dt;
            sxX += dt * (s[1] - meanX);
            sxY += dt * (s[2] - meanY);
        }
        if (sxx < 1e-9) return new Pose(0, 0, 0);

        double vx = sxX / sxx, vy = sxY / sxx;
        double sp = Math.hypot(vx, vy);
        if (sp > MAX_BALL_SPEED_IN_S && sp > 1e-9) {
            double s = MAX_BALL_SPEED_IN_S / sp;
            vx *= s; vy *= s;
        }
        return new Pose(vx, vy, 0);
    }

    private void pruneStale(double nowSec) {
        for (int i = tracks.size() - 1; i >= 0; i--) {
            if (nowSec - tracks.get(i).lastSeenSec > STALE_SEC) tracks.remove(i);
        }
    }

    private boolean outOfField(double x, double y) {
        return x < -FIELD_MARGIN_IN || x > FIELD_SIZE_IN + FIELD_MARGIN_IN
                || y < -FIELD_MARGIN_IN || y > FIELD_SIZE_IN + FIELD_MARGIN_IN;
    }

    private static Artifact colorFromClassName(String className) {
        if (className == null) return Artifact.NONE;
        String c = className.toLowerCase();
        if (c.contains(PURPLE_CLASS.toLowerCase())) return Artifact.PURPLE;
        if (c.contains(GREEN_CLASS.toLowerCase())) return Artifact.GREEN;
        return Artifact.NONE;
    }

    private static double[] cross(double[] a, double[] b) {
        return new double[]{
                a[1] * b[2] - a[2] * b[1],
                a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0]
        };
    }

    /** Currently tracked balls (live list view; copy if you need to retain it). */
    public List<Ball> getBalls() {
        return new ArrayList<>(tracks);
    }

    public void reset() {
        tracks.clear();
    }
}
