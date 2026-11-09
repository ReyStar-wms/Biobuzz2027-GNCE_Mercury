package org.firstinspires.ftc.teamcode.opmodes.auto.base;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

/**
 * Checks the two turret servos and the absolute encoder on the turret output.
 *
 * <p>The encoder is the part that fails quietly: {@code Turret} derives everything from it, so a
 * dead analog line leaves the turret aiming at a fixed angle forever. The nudge test commands a few
 * degrees of servo travel and confirms the encoder moved by roughly the gear ratio, which is what
 * separates a dead encoder from a slipping gear from a servo with no power.
 *
 * <p>Servo direction and PWM range are set to the same values {@code Turret} uses. Moves are small
 * and relative to wherever the turret happens to be.
 */
@Autonomous(name = "03 Check: turret", group = "Device Check")
public class TurretCheck extends DeviceCheckOpMode {

    /** Reference voltage Turret passes to its AbsoluteAnalogEncoder. */
    private static final double ENCODER_FULL_SCALE_VOLTS = 3.3;

    /** Servo units to move. 0.02 of a 348 deg range is about 7 deg of servo travel. */
    private static final double NUDGE = 0.02;

    /** Servo degrees across the full servo range, from Turret.SERVO_RANGE. */
    private static final double SERVO_RANGE_DEG = 348;

    /** From Turret.GEAR_RATIO: turret degrees = servo degrees / ratio. */
    private static final double GEAR_RATIO = (double) 99 / 24 * 20 / 96;

    @Override
    protected String safetyNote() {
        return "The turret will move a few degrees. Check nothing is in its way.";
    }

    @Override
    protected void runChecks() {
        section("Encoder");
        AnalogInput encoder = device(AnalogInput.class, Devices.TURRET_ENCODER);
        if (encoder != null) {
            check("encoder reading", () -> String.format("%.3f V of %.2f V -> %.1f deg",
                    encoder.getVoltage(), encoder.getMaxVoltage(), angleOf(encoder)));

            double volts = encoder.getVoltage();
            if (volts < 0.02) {
                // Can legitimately happen at one end of travel, but a floating analog line reads
                // the same way, which is the far more common cause.
                warn("encoder", String.format("%.3f V - check the analog cable if it never changes", volts));
            }
            observe("turn the turret by hand", 6000, () ->
                    String.format("%.3f V -> %.1f deg", encoder.getVoltage(), angleOf(encoder)));
        }

        section("Servos");
        ServoImplEx servo1 = device(ServoImplEx.class, Devices.TURRET_SERVO_1);
        ServoImplEx servo2 = device(ServoImplEx.class, Devices.TURRET_SERVO_2);
        if (servo1 == null || servo2 == null) {
            return;
        }

        check("configure both", () -> {
            configure(servo1);
            configure(servo2);
            return String.format("reverse, pwm 500-2500 us, enabled %s / %s",
                    servo1.isPwmEnabled(), servo2.isPwmEnabled());
        });

        check("both report the same position", () -> {
            double p1 = servo1.getPosition();
            double p2 = servo2.getPosition();
            // They are geared to one output, so a big disagreement means one is not following.
            require(Math.abs(p1 - p2) < 0.1, String.format(
                    "ts1 at %.3f but ts2 at %.3f - they drive the same gear", p1, p2));
            return String.format("ts1 %.3f, ts2 %.3f", p1, p2);
        });

        if (encoder == null) {
            skipped("nudge", "no encoder to measure the move with");
            return;
        }
        if (!confirm("nudge the turret a few degrees")) {
            return;
        }

        section("Nudge");
        double start = servo1.getPosition();
        nudge("forward", servo1, servo2, encoder, clamp(start + NUDGE));
        nudge("back", servo1, servo2, encoder, clamp(start));
    }

    private void nudge(String label, ServoImplEx s1, ServoImplEx s2, AnalogInput encoder, double target) {
        check("nudge " + label, () -> {
            double before = angleOf(encoder);
            s1.setPosition(target);
            s2.setPosition(target);
            sleep(600);
            double after = angleOf(encoder);
            double moved = Math.abs(shortestAngle(after - before));
            double expected = NUDGE * SERVO_RANGE_DEG / GEAR_RATIO;

            require(moved > expected * 0.25, String.format(
                    "encoder moved %.1f deg, expected about %.1f - servo not driving, or encoder stuck",
                    moved, expected));
            return String.format("servo to %.3f, encoder %.1f -> %.1f deg (moved %.1f, expected ~%.1f)",
                    target, before, after, moved, expected);
        });
    }

    private static void configure(ServoImplEx servo) {
        servo.setDirection(Servo.Direction.REVERSE);
        servo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        servo.setPwmEnable();
    }

    private static double angleOf(AnalogInput encoder) {
        return encoder.getVoltage() / ENCODER_FULL_SCALE_VOLTS * 360.0;
    }

    /** The encoder wraps at 360, so a move across the seam must not read as 350 deg. */
    private static double shortestAngle(double degrees) {
        double wrapped = degrees % 360.0;
        if (wrapped > 180.0) {
            wrapped -= 360.0;
        }
        if (wrapped < -180.0) {
            wrapped += 360.0;
        }
        return wrapped;
    }

    private static double clamp(double position) {
        return Math.max(0.0, Math.min(1.0, position));
    }
}
