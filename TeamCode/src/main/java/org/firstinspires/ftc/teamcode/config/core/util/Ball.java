package org.firstinspires.ftc.teamcode.config.core.util;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.config.core.util.robothelper.Artifact;

import java.util.ArrayDeque;

/**
 * A single tracked artifact (ball) in field (Pedro) coordinates. Produced and updated by
 * {@link BallLocalizer}; consumed by {@link BallPathPlanner}.
 *
 * <p>Both {@link #position} (inches) and {@link #velocity} (inches/sec, stored as an x/y
 * {@link Pose} with an unused heading of 0) are Pedro {@link Pose}s so the tracker and
 * planner can use Pose vector math ({@code plus}/{@code minus}/{@code times}/
 * {@code distanceFrom}) directly. {@code Pose} is immutable, so an update replaces the
 * reference rather than mutating in place.
 */
public class Ball {
    public final int id;
    public Artifact color;

    public Pose position;   // field position, inches
    public Pose velocity;   // field velocity as (vx, vy), inches/sec (heading unused)

    public double lastSeenSec;  // robot clock time of the most recent update
    public double confidence;   // detector confidence of the most recent detection, 0..1

    // Recent raw (time, x, y) measurements, used to fit a smooth velocity by regression.
    public final ArrayDeque<double[]> history = new ArrayDeque<>();

    public Ball(int id, Artifact color, Pose position, double nowSec, double confidence) {
        this.id = id;
        this.color = color;
        this.position = position;
        this.velocity = new Pose(0, 0, 0);
        this.lastSeenSec = nowSec;
        this.confidence = confidence;
    }

    /** Record a raw measurement for velocity fitting. */
    public void pushSample(double t, double x, double y) {
        history.addLast(new double[]{t, x, y});
    }

    /** Predicted position {@code dtSec} into the future, assuming constant velocity. */
    public Pose predict(double dtSec) {
        return position.plus(velocity.times(dtSec));
    }

    public double speed() {
        return Math.hypot(velocity.getX(), velocity.getY());
    }
}
