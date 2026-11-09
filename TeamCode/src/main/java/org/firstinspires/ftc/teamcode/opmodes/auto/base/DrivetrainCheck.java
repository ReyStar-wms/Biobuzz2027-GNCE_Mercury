package org.firstinspires.ftc.teamcode.opmodes.auto.base;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.config.subsystems.Lift;

/**
 * Spins each drive motor on its own and reports whether it turned, which way, and how much current
 * it drew. This is the check for "one wheel is not driving" and for "the encoder cable is out" -
 * both show up as a motor that draws current but reports no ticks, or vice versa.
 *
 * <p>Directions are reported, not asserted: Pedro applies its own inversions in
 * {@code pedroPathing.Constants}, so what matters here is that each motor moves and its encoder
 * counts. Compare the four signs against each other - with the robot up on blocks, all four should
 * report the same sign for a positive power if they are wired alike.
 */
@Autonomous(name = "01 Check: drivetrain", group = "Device Check")
public class DrivetrainCheck extends DeviceCheckOpMode {

    private static final double TEST_POWER = 0.3;
    private static final long SPIN_MS = 600;
    private static final int MIN_TICKS = 20;

    @Override
    protected String safetyNote() {
        return "WHEELS WILL SPIN - put the robot up on blocks first.";
    }

    @Override
    protected void runChecks() {
        section("Lift clutch");
        Servo pto = device(Servo.class, Devices.PTO_SERVO);
        if (pto != null) {
            check("disengage PTO", () -> {
                // The lift drives off these same motors, so park the clutch where the lift is
                // inactive before spinning anything.
                pto.setPosition(Lift.CLUTCH_PULLED);
                sleep(400);
                return String.format("clutch to %.2f (lift disengaged)", Lift.CLUTCH_PULLED);
            });
        } else {
            warn("PTO", "no clutch servo - the lift may be geared to the wheels");
        }

        for (String name : Devices.DRIVE_MOTORS) {
            section("Motor " + name);
            DcMotorEx motor = device(DcMotorEx.class, name);
            if (motor == null) {
                continue;
            }

            check("configure", () -> {
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                return "open loop, brake, direction " + motor.getDirection();
            });

            if (!confirm("spin " + name)) {
                continue;
            }
            spin(name, motor);
        }
    }

    private void spin(String name, DcMotorEx motor) {
        final int[] delta = new int[1];
        final double[] peakCurrent = new double[1];

        check("spin " + name, () -> {
            int start = motor.getCurrentPosition();
            motor.setPower(TEST_POWER);
            try {
                long end = System.currentTimeMillis() + SPIN_MS;
                while (opModeIsActive() && System.currentTimeMillis() < end) {
                    peakCurrent[0] = Math.max(peakCurrent[0], motor.getCurrent(CurrentUnit.AMPS));
                    sleep(25);
                }
            } finally {
                motor.setPower(0);
            }
            sleep(250); // let it coast to a stop before reading the encoder
            delta[0] = motor.getCurrentPosition() - start;
            return String.format("%+d ticks at %.0f%% power, peak %.2f A",
                    delta[0], TEST_POWER * 100, peakCurrent[0]);
        });

        if (Math.abs(delta[0]) < MIN_TICKS) {
            if (peakCurrent[0] > 1.0) {
                warn(name + " encoder", String.format(
                        "drew %.2f A but only moved %+d ticks - encoder cable or a stalled wheel",
                        peakCurrent[0], delta[0]));
            } else {
                warn(name + " output", String.format(
                        "%+d ticks and only %.2f A - power cable, or the motor is not turning",
                        delta[0], peakCurrent[0]));
            }
        }
    }
}
