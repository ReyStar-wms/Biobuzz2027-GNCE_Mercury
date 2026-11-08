package org.firstinspires.ftc.teamcode.config.core.util;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.config.core.util.robothelper.Artifact;

import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;

/**
 * Picks the best three balls to collect and builds a Pedro {@link PathChain} that drives
 * the robot through all three, accounting for each ball's velocity (it aims at where the
 * ball <i>will be</i> when the robot arrives, not where it is now).
 *
 * <p><b>Optimality:</b> with only a handful of candidate balls this is a tiny problem, so
 * we brute-force it. We evaluate every ordered selection of balls (length 1..goal),
 * simulating the run leg-by-leg: each leg's arrival time advances the clock, and the next
 * ball is predicted forward to that cumulative time before we solve its intercept. The route
 * that collects the goal number of balls in the least time wins. Brute force (rather than
 * greedy nearest-neighbor) matters because a slightly farther first ball can set up two much
 * shorter later legs.
 *
 * <p><b>Corridor sweep:</b> the intake is wide ({@link #INTAKE_HALF_WIDTH_IN} on each side),
 * so any ball within that distance of a leg's line is swept up in passing — no separate
 * waypoint needed. The search counts these free pickups, so it prefers a route whose legs
 * graze extra balls and can hit the goal count with fewer explicit stops (e.g. a 2-leg route
 * that sweeps a third ball beats a 3-leg route). Only the explicitly-targeted balls become
 * waypoints; the swept ones are collected en route.
 *
 * <p>Intercept per leg is solved the same iterative way as the shooter's move-shot lead
 * (see {@link ShotPlanner}): estimate travel time from distance, advance the ball by
 * velocity*time, repeat.
 *
 * <p><b>Field safeguard:</b> every waypoint the robot is told to drive to is the robot's
 * <i>center</i>, so it is clamped to stay {@link #WALL_MARGIN_IN} inside the field. That
 * keeps the chassis (and especially the forward-reaching intake) from being commanded past
 * a wall, which is what made the robot drive into the boundary and get stuck. Because the
 * legs are straight lines and the safe region is a convex box, clamping the endpoints keeps
 * the whole path inside.
 *
 * <p><b>Intake-aware targeting:</b> the intake mouth sits {@link #INTAKE_FORWARD_IN} ahead of
 * the center of rotation, so the center target is aimed that far short of the ball to land the
 * mouth on it. Lateral precision is not needed (see corridor sweep above), so the field clamp
 * can move a target sideways freely without missing.
 *
 * <h3>Usage</h3>
 * <pre>{@code
 * BallPathPlanner.Plan plan = planner.plan(follower, follower.getPose(),
 *                                          localizer.getBalls(), Artifact.PURPLE);
 * if (plan != null) follower.followPath(plan.path, true);
 * }</pre>
 */
@Configurable
public class BallPathPlanner {

    /** Assumed average travel speed used to estimate leg times, inches/sec. */
    public static double TRAVEL_SPEED_IN_S = 40.0;
    /** Iterations of the per-leg intercept solver. */
    public static int INTERCEPT_ITERS = 3;
    /** How many balls to string together into one path. */
    public static int MAX_BALLS_IN_PATH = 3;
    /** Don't lead a moving ball more than this far from its current spot, inches. */
    public static double MAX_LEAD_IN = 48.0;

    /** Field side length, inches (square field). */
    public static double FIELD_SIZE_IN = 141.5;
    /** Keep the robot center at least this far from any wall, inches. Cover the front extent
     *  of the chassis + intake so nothing is ever commanded past the boundary. */
    public static double WALL_MARGIN_IN = 11.0;
    /** Forward distance from the center of rotation to the intake mouth, inches. The center
     *  is aimed this far short of the ball so the mouth (not the center) lands on it. */
    public static double INTAKE_FORWARD_IN = 4.5;
    /** Half the intake's left-to-right width, inches. A ball within this distance of a leg's
     *  line is swept up in passing. Trimmed below the true 6 in for margin against missing. */
    public static double INTAKE_HALF_WIDTH_IN = 4.5;
    /** Drop path legs shorter than this, inches (degenerate segments hang the follower). */
    public static double MIN_SEGMENT_IN = 1.0;

    /** Result of a plan: the path to follow plus the chosen targets and their intercept points. */
    public static class Plan {
        public final PathChain path;
        public final List<Ball> targets;        // chosen balls, in collection order
        public final List<Pose> interceptPoses; // where the robot meets each ball (heading set)
        public final double estTimeSec;

        public Plan(PathChain path, List<Ball> targets, List<Pose> interceptPoses, double estTimeSec) {
            this.path = path;
            this.targets = targets;
            this.interceptPoses = interceptPoses;
            this.estTimeSec = estTimeSec;
        }
    }

    /**
     * Plan a path that collects up to {@link #MAX_BALLS_IN_PATH} balls in the least time,
     * counting balls swept up along each leg's corridor (see class docs).
     *
     * @param follower    used only to obtain a {@link PathBuilder}
     * @param robotPose   current robot pose (start of the run)
     * @param balls       candidate balls (field coords + velocity) from {@link BallLocalizer}
     * @param colorFilter only consider this color; pass {@code null} or {@link Artifact#NONE} for any
     * @return a {@link Plan}, or {@code null} if there are no usable balls
     */
    public Plan plan(Follower follower, Pose robotPose, List<Ball> balls, Artifact colorFilter) {
        List<Ball> candidates = new ArrayList<>();
        for (Ball b : balls) {
            if (colorFilter == null || colorFilter == Artifact.NONE || b.color == colorFilter) {
                candidates.add(b);
            }
        }
        if (candidates.isEmpty()) return null;

        int goal = Math.min(MAX_BALLS_IN_PATH, candidates.size());

        // Search every ordered selection (length 1..goal) for the one that collects the most
        // balls (capped at goal, counting corridor sweeps) in the least time.
        Best best = new Best();
        search(robotPose, candidates, new ArrayList<>(), goal, best);
        if (best.sequence == null) return null;

        // Re-evaluate the winner to recover its stop poses and full collected set.
        Eval win = evaluate(robotPose, best.sequence, candidates);

        // Build the Pedro path: straight legs through the stop poses, heading along travel.
        // Skip any near zero-length leg — a degenerate BezierLine can hang the follower.
        PathBuilder builder = follower.pathBuilder();
        List<Pose> waypoints = new ArrayList<>();
        Pose prev = robotPose;
        for (Pose stop : win.stops) {
            if (prev.distanceFrom(stop) < MIN_SEGMENT_IN) continue;
            builder.addPath(new BezierLine(prev, stop))
                    .setLinearHeadingInterpolation(prev.getHeading(), stop.getHeading());
            prev = stop;
            waypoints.add(stop);
        }
        if (waypoints.isEmpty()) return null; // already on the ball(s); nothing to drive

        return new Plan(builder.build(), new ArrayList<>(win.collected), waypoints, win.cost);
    }

    private static class Best {
        List<Ball> sequence = null;
        int collected = -1;                    // capped at goal
        double cost = Double.POSITIVE_INFINITY;
    }

    /**
     * Recursively try every ordered selection of primary targets, scoring each by how many
     * balls it collects (capped at {@code goal}, including corridor sweeps) then by time.
     */
    private void search(Pose start, List<Ball> pool, List<Ball> chosen, int goal, Best best) {
        if (!chosen.isEmpty()) {
            Eval e = evaluate(start, chosen, pool);
            int capped = Math.min(e.collected.size(), goal);
            if (capped > best.collected || (capped == best.collected && e.cost < best.cost)) {
                best.collected = capped;
                best.cost = e.cost;
                best.sequence = new ArrayList<>(chosen);
            }
            // Enough balls already; adding more legs only adds time for the same capped count.
            if (e.collected.size() >= goal) return;
        }
        if (chosen.size() >= goal) return;
        for (Ball b : pool) {
            if (chosen.contains(b)) continue;
            chosen.add(b);
            search(start, pool, chosen, goal, best);
            chosen.remove(chosen.size() - 1);
        }
    }

    /** Outcome of driving a primary-target sequence: time, balls collected, and stop poses. */
    private static class Eval {
        final double cost;
        final LinkedHashSet<Ball> collected;
        final List<Pose> stops;

        Eval(double cost, LinkedHashSet<Ball> collected, List<Pose> stops) {
            this.cost = cost;
            this.collected = collected;
            this.stops = stops;
        }
    }

    /**
     * Simulate driving to each primary target in order. Each leg sweeps up any pool ball whose
     * (predicted) position lies within {@link #INTAKE_HALF_WIDTH_IN} of the line from the leg
     * start to the targeted ball.
     */
    private Eval evaluate(Pose start, List<Ball> sequence, List<Ball> pool) {
        double tCum = 0;
        Pose from = start;
        LinkedHashSet<Ball> collected = new LinkedHashSet<>();
        List<Pose> stops = new ArrayList<>();

        for (Ball primary : sequence) {
            Intercept hit = intercept(from, primary, tCum);
            double dxh = hit.point.getX() - from.getX();
            double dyh = hit.point.getY() - from.getY();
            double heading = (Math.hypot(dxh, dyh) > 1e-3) ? Math.atan2(dyh, dxh) : from.getHeading();
            Pose stop = hit.point.withHeading(heading);

            // Anything within the intake's width of the swept line (start -> ball) is collected.
            for (Ball other : pool) {
                if (collected.contains(other)) continue;
                if (pointNearSegment(other.predict(tCum), from, hit.ballPoint, INTAKE_HALF_WIDTH_IN)) {
                    collected.add(other);
                }
            }
            collected.add(primary);

            stops.add(stop);
            tCum += hit.timeSec;
            from = stop;
        }
        return new Eval(tCum, collected, stops);
    }

    /** True if point {@code p} is within {@code tol} of segment [a, b]. */
    private static boolean pointNearSegment(Pose p, Pose a, Pose b, double tol) {
        double abx = b.getX() - a.getX(), aby = b.getY() - a.getY();
        double len2 = abx * abx + aby * aby;
        double t = len2 < 1e-9 ? 0 : ((p.getX() - a.getX()) * abx + (p.getY() - a.getY()) * aby) / len2;
        t = t < 0 ? 0 : (t > 1 ? 1 : t);
        double cx = a.getX() + t * abx, cy = a.getY() + t * aby;
        return Math.hypot(p.getX() - cx, p.getY() - cy) <= tol;
    }

    /** A leg result: where the robot stops ({@code point}), where the ball is ({@code ballPoint}), and the leg time. */
    private static class Intercept {
        final Pose point;      // robot-center stop pose (intake-offset + field-clamped)
        final Pose ballPoint;  // predicted ball location (used to aim the intake)
        final double timeSec;

        Intercept(Pose point, Pose ballPoint, double timeSec) {
            this.point = point;
            this.ballPoint = ballPoint;
            this.timeSec = timeSec;
        }
    }

    /**
     * Solve where to meet ball {@code b}, starting from {@code from} after {@code tOffset}
     * seconds have already elapsed (so the ball is predicted from its tOffset position). The
     * returned stop point is aimed {@link #INTAKE_FORWARD_IN} short of the ball and clamped
     * inside the field, so the leg time reflects the distance the robot actually drives.
     */
    private Intercept intercept(Pose from, Ball b, double tOffset) {
        double speed = Math.max(1e-3, TRAVEL_SPEED_IN_S);

        // Ball's position at the moment this leg begins.
        Pose ballStart = b.predict(tOffset);

        Pose ballPoint = ballStart;
        Pose stop = ballStart;
        double legTime = 0;
        for (int i = 0; i < Math.max(1, INTERCEPT_ITERS); i++) {
            legTime = from.distanceFrom(stop) / speed;

            Pose lead = b.velocity.times(legTime);
            double leadMag = Math.hypot(lead.getX(), lead.getY());
            if (leadMag > MAX_LEAD_IN && leadMag > 1e-6) {
                lead = lead.times(MAX_LEAD_IN / leadMag);
            }
            ballPoint = ballStart.plus(lead);
            stop = clampToField(approachStop(from, ballPoint));
        }
        return new Intercept(stop, ballPoint, legTime);
    }

    /**
     * Center-of-rotation target that lands the intake mouth on the ball. The mouth is
     * {@link #INTAKE_FORWARD_IN} ahead of center, so the center stops that far short of the ball.
     */
    private static Pose approachStop(Pose from, Pose ballPoint) {
        double dx = ballPoint.getX() - from.getX();
        double dy = ballPoint.getY() - from.getY();
        double d = Math.hypot(dx, dy);
        if (d <= INTAKE_FORWARD_IN) return ballPoint; // ball already within intake reach
        return new Pose(ballPoint.getX() - dx / d * INTAKE_FORWARD_IN, ballPoint.getY() - dy / d * INTAKE_FORWARD_IN, 0);
    }

    /** Clamp a robot-center target so the chassis stays inside the field walls. */
    private static Pose clampToField(Pose p) {
        double lo = WALL_MARGIN_IN;
        double hi = FIELD_SIZE_IN - WALL_MARGIN_IN;
        return new Pose(clamp(p.getX(), lo, hi), clamp(p.getY(), lo, hi), 0);
    }

    private static double clamp(double v, double lo, double hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }
}
