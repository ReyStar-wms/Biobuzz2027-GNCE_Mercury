package org.firstinspires.ftc.teamcode.opmodes.auto.base;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.config.subsystems.Intake;

/**
 * Runs the intake roller and the transfer, forwards then backwards, and reports current draw and
 * encoder movement for each. Also parks the pitch servo at the position {@code Intake} uses.
 *
 * <p>Current draw is the useful number here: a roller that spins free draws very little, and one
 * that is jammed draws a lot without turning.
 */
@Autonomous(name = "04 Check: intake and transfer", group = "Device Check")
public class IntakeCheck extends DeviceCheckOpMode {

    private static final double TEST_POWER = 0.4;
    private static final long SPIN_MS = 800;

    @Override
    protected String safetyNote() {
        return "The intake and transfer will run. Keep hands and loose artifacts clear.";
    }

    @Override
    protected void runChecks() {
        section("Pitch servo");
        Servo pitch = device(Servo.class, Devices.INTAKE_PITCH_SERVO);
        if (pitch != null && confirm("move the intake pitch to its stored position")) {
            check("pitch to down position", () -> {
                pitch.setPosition(Intake.INTAKE_DOWN);
                sleep(500);
                return String.format("commanded %.3f (Intake.INTAKE_DOWN)", Intake.INTAKE_DOWN);
            });
            if (Math.abs(Intake.INTAKE_DOWN - Intake.INTAKE_UP) < 1e-9) {
                warn("pitch travel", String.format(
                        "INTAKE_DOWN and INTAKE_UP are both %.3f, so the pitch never actually moves",
                        Intake.INTAKE_DOWN));
            }
        }

        section("Intake roller");
        DcMotorEx intake = device(DcMotorEx.class, Devices.INTAKE_MOTOR);
        if (intake != null) {
            configure(Devices.INTAKE_MOTOR, intake);
            if (confirm("run the intake roller both ways")) {
                spin("intake forward", intake, TEST_POWER);
                spin("intake reverse", intake, -TEST_POWER);
            }
        }

        section("Transfer");
        DcMotorEx transfer = device(DcMotorEx.class, Devices.TRANSFER_MOTOR);
        if (transfer != null) {
            configure(Devices.TRANSFER_MOTOR, transfer);
            if (confirm("run the transfer both ways")) {
                spin("transfer forward", transfer, TEST_POWER);
                spin("transfer reverse", transfer, -TEST_POWER);
            }
        }
    }

    private void configure(String name, DcMotorEx motor) {
        check("configure " + name, () -> {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            return "open loop, float, direction " + motor.getDirection();
        });
    }

    private void spin(String label, DcMotorEx motor, double power) {
        final double[] peak = new double[1];
        final int[] delta = new int[1];

        check(label, () -> {
            int start = motor.getCurrentPosition();
            motor.setPower(power);
            try {
                long end = System.currentTimeMillis() + SPIN_MS;
                while (opModeIsActive() && System.currentTimeMillis() < end) {
                    peak[0] = Math.max(peak[0], motor.getCurrent(CurrentUnit.AMPS));
                    sleep(25);
                }
            } finally {
                motor.setPower(0);
            }
            sleep(200);
            delta[0] = motor.getCurrentPosition() - start;
            require(peak[0] > 0.05, String.format(
                    "drew only %.2f A - the motor is probably not powered", peak[0]));
            return String.format("%+d ticks, peak %.2f A", delta[0], peak[0]);
        });

        if (delta[0] == 0) {
            warn(label + " encoder", "no ticks - this motor may have no encoder wired, which is fine"
                    + " if the subsystem never reads its position");
        }
    }
}
