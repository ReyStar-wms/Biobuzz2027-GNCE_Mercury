package org.firstinspires.ftc.teamcode.opmodes.auto.base;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config.subsystems.Lift;

/**
 * Checks the lift's power take-off clutch servo, and leaves it where the drivetrain can run.
 *
 * <p>The lift has no motors of its own - {@code Lift} drives the four drivetrain motors through this
 * clutch, which is why {@code DrivetrainCheck} pulls the clutch before spinning anything. So the
 * only thing to check off-field is the clutch servo itself: it should move between
 * {@code Lift.CLUTCH_PULLED} (lift idle, wheels drive) and {@code Lift.CLUTCH_MESHED} (lift
 * engaged), and you should hear or feel it mesh.
 *
 * <p>Extending the lift is deliberately not automated here: with the clutch meshed, driving the
 * motors lifts the robot, which is not something a bench check should do on its own. Use the
 * {@code LiftManualControl} teleop for that, on the field, with someone spotting.
 */
@Autonomous(name = "09 Check: lift clutch (PTO)", group = "Device Check")
public class LiftPtoCheck extends DeviceCheckOpMode {

    @Override
    protected String safetyNote() {
        return "The clutch servo will move. The lift itself is NOT driven by this check.";
    }

    @Override
    protected void runChecks() {
        section("Device");
        Servo pto = device(Servo.class, Devices.PTO_SERVO);
        if (pto == null) {
            return;
        }

        check("positions", () -> String.format("pulled (lift idle) %.2f, meshed (lift driven) %.2f",
                Lift.CLUTCH_PULLED, Lift.CLUTCH_MESHED));

        if (Math.abs(Lift.CLUTCH_PULLED - Lift.CLUTCH_MESHED) < 0.05) {
            warn("travel", "the two clutch positions are nearly identical - the PTO cannot shift");
        }

        if (!confirm("shift the clutch: pulled - meshed - pulled")) {
            return;
        }

        section("Shift");
        move("pulled", pto, Lift.CLUTCH_PULLED);
        move("meshed", pto, Lift.CLUTCH_MESHED);
        move("pulled again", pto, Lift.CLUTCH_PULLED);

        check("left disengaged", () -> {
            // Important: whatever happens above, do not leave the robot with the lift geared in.
            pto.setPosition(Lift.CLUTCH_PULLED);
            sleep(400);
            return String.format("clutch at %.2f, the wheels drive again", pto.getPosition());
        });
    }

    private void move(String label, Servo pto, double position) {
        check("clutch " + label, () -> {
            pto.setPosition(position);
            sleep(Math.max(600, Lift.ENGAGING_DELAY * 2));
            return String.format("commanded %.2f, servo reports %.2f", position, pto.getPosition());
        });
    }
}
