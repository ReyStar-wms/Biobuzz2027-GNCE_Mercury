package org.firstinspires.ftc.teamcode.config.hardware;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * Wraps a {@link Servo} and only forwards {@code setPosition} to the hardware when the commanded
 * position actually changes (beyond an optional threshold). A positional servo holds its last PWM,
 * so re-sending the same value every loop is a wasted Lynx command -- this skips those. Mirrors
 * {@link CachedMotor}. Construct from any Servo (incl. ServoImplEx after its PWM range is configured).
 */
public class CachedServo {
    private final Servo servo;
    private double cachePositionThreshold;
    private double lastPosition = Double.NaN;

    public CachedServo(Servo servo) {
        this(servo, 0.0);
    }

    public CachedServo(Servo servo, double cachePositionThreshold) {
        this.servo = servo;
        this.cachePositionThreshold = cachePositionThreshold;
    }

    public void setPosition(double position) {
        if (Double.isNaN(lastPosition)
                || Math.abs(position - lastPosition) > cachePositionThreshold) {
            servo.setPosition(position);
            lastPosition = position;
        }
    }

    /** Force the next setPosition to write regardless of the cache (e.g. after re-enabling PWM). */
    public void invalidateCache() {
        lastPosition = Double.NaN;
    }

    public double getLastPosition() {
        return lastPosition;
    }

    public void setCachePositionThreshold(double cachePositionThreshold) {
        this.cachePositionThreshold = cachePositionThreshold;
    }

    public Servo getServo() {
        return servo;
    }
}
