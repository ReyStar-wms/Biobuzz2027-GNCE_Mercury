package org.firstinspires.ftc.teamcode.opmodes.auto.base;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.config.subsystems.Shooter;

/**
 * Checks the two flywheel motors and the hood servo.
 *
 * <p>The flywheel runs at a low fixed power - well under shooting speed - just long enough to read
 * velocity off both encoders. Both motors drive the same wheel, so their velocities should agree in
 * magnitude; a large mismatch means one is not pulling, and zero velocity with current draw means
 * that motor's encoder is not wired.
 *
 * <p>The hood sweep uses the same angle-to-servo mapping {@code Shooter} uses, so it stays inside
 * the mechanical limits even if those constants are retuned.
 */
@Autonomous(name = "05 Check: shooter", group = "Device Check")
public class ShooterCheck extends DeviceCheckOpMode {

    /** Low enough to be safe on a bench, high enough to read a clear velocity. */
    private static final double TEST_POWER = 0.25;
    private static final long SPIN_MS = 1500;

    @Override
    protected String safetyNote() {
        return "THE FLYWHEEL WILL SPIN at 25% power. Clear the shooter and stand clear of its path.";
    }

    @Override
    protected void runChecks() {
        section("Flywheel");
        DcMotorEx m1 = device(DcMotorEx.class, Devices.FLYWHEEL_1);
        DcMotorEx m2 = device(DcMotorEx.class, Devices.FLYWHEEL_2);

        if (m1 != null && m2 != null) {
            check("configure both", () -> {
                for (DcMotorEx motor : new DcMotorEx[]{m1, m2}) {
                    motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    // Never brake a flywheel: let it coast down.
                    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                }
                return "open loop, float, directions " + m1.getDirection() + " / " + m2.getDirection();
            });

            if (confirm("spin the flywheel at 25% for 1.5 s")) {
                spinFlywheel(m1, m2);
            }
        }

        section("Hood");
        Servo hood = device(Servo.class, Devices.HOOD_SERVO);
        if (hood != null && confirm("sweep the hood through its range")) {
            sweepHood(hood, Shooter.IDLE_HOOD_ANGLE_DEG);
            sweepHood(hood, Shooter.MAX_HOOD_ANGLE_DEG);
            sweepHood(hood, Shooter.MIN_HOOD_ANGLE_DEG);
            sweepHood(hood, Shooter.IDLE_HOOD_ANGLE_DEG);
        }
    }

    private void spinFlywheel(DcMotorEx m1, DcMotorEx m2) {
        final double[] v1 = new double[1];
        final double[] v2 = new double[1];
        final double[] peak1 = new double[1];
        final double[] peak2 = new double[1];

        check("spin up", () -> {
            m1.setPower(TEST_POWER);
            m2.setPower(TEST_POWER);
            try {
                long end = System.currentTimeMillis() + SPIN_MS;
                while (opModeIsActive() && System.currentTimeMillis() < end) {
                    v1[0] = m1.getVelocity();
                    v2[0] = m2.getVelocity();
                    peak1[0] = Math.max(peak1[0], m1.getCurrent(CurrentUnit.AMPS));
                    peak2[0] = Math.max(peak2[0], m2.getCurrent(CurrentUnit.AMPS));
                    status(String.format("spinning: %.0f / %.0f ticks/s", v1[0], v2[0]));
                    sleep(50);
                }
            } finally {
                m1.setPower(0);
                m2.setPower(0);
            }
            return String.format("%s %.0f ticks/s at %.2f A, %s %.0f ticks/s at %.2f A",
                    Devices.FLYWHEEL_1, v1[0], peak1[0],
                    Devices.FLYWHEEL_2, v2[0], peak2[0]);
        });

        check("both motors pulling", () -> {
            require(peak1[0] > 0.1, Devices.FLYWHEEL_1 + " drew almost no current - not powered");
            require(peak2[0] > 0.1, Devices.FLYWHEEL_2 + " drew almost no current - not powered");
            return String.format("%.2f A and %.2f A", peak1[0], peak2[0]);
        });

        double speed1 = Math.abs(v1[0]);
        double speed2 = Math.abs(v2[0]);
        if (speed1 < 50 && speed2 < 50) {
            warn("flywheel encoders", "neither motor reported velocity - check both encoder cables");
        } else if (speed1 < 50 || speed2 < 50) {
            warn("flywheel encoder", String.format(
                    "%s read %.0f ticks/s but %s read %.0f - one encoder cable is out",
                    Devices.FLYWHEEL_1, speed1, Devices.FLYWHEEL_2, speed2));
        } else {
            double ratio = Math.min(speed1, speed2) / Math.max(speed1, speed2);
            if (ratio < 0.8) {
                warn("flywheel match", String.format(
                        "%.0f vs %.0f ticks/s on one wheel - one motor is not pulling its share",
                        speed1, speed2));
            }
            if (Math.signum(v1[0]) != Math.signum(v2[0])) {
                warn("flywheel directions", String.format(
                        "velocities have opposite signs (%.0f and %.0f) - they are fighting each other",
                        v1[0], v2[0]));
            }
        }

        check("coast down", () -> {
            sleep(1500);
            double after = Math.max(Math.abs(m1.getVelocity()), Math.abs(m2.getVelocity()));
            return String.format("%.0f ticks/s 1.5 s after power off", after);
        });
    }

    private void sweepHood(Servo hood, double hoodAngleDeg) {
        check(String.format("hood to %.1f deg", hoodAngleDeg), () -> {
            double position = hoodAngleToServoPos(hoodAngleDeg);
            hood.setPosition(position);
            sleep(600);
            return String.format("servo %.3f", position);
        });
    }

    /** Copy of Shooter.hoodAngleToServoPos, which is private but driven by public constants. */
    private static double hoodAngleToServoPos(double hoodAngleDeg) {
        double clipped = Range.clip(hoodAngleDeg, Shooter.MIN_HOOD_ANGLE_DEG, Shooter.MAX_HOOD_ANGLE_DEG);
        double fromMax = Shooter.MAX_HOOD_ANGLE_DEG - clipped;
        return Shooter.HOOD_MAX_SERVO_POS - fromMax * Shooter.HOOD_GEAR_RATIO / Shooter.HOOD_SERVO_RANGE;
    }
}
