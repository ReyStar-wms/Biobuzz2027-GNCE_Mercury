package org.firstinspires.ftc.teamcode.opmodes.auto.base;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config.subsystems.Door;

/**
 * Cycles the shooter door between the open and closed positions {@code Door} uses, holding each one
 * long enough to watch. Both positions come from {@code Door}'s public constants, so this moves to
 * exactly where the match code would.
 *
 * <p>A servo that buzzes without moving, or only reaches one end, shows up here rather than during
 * a shot sequence.
 */
@Autonomous(name = "07 Check: shooter door", group = "Device Check")
public class DoorCheck extends DeviceCheckOpMode {

    @Override
    protected String safetyNote() {
        return "The door servo will cycle. Keep fingers out of the gate.";
    }

    @Override
    protected void runChecks() {
        section("Device");
        Servo door = device(Servo.class, Devices.DOOR_SERVO);
        if (door == null) {
            return;
        }

        check("positions", () -> String.format("closed %.3f, open %.3f, offset %.3f, %d ms travel",
                Door.CLOSED_POS, Door.OPEN_POS, Door.OFFSET_POS, Door.ACTUATION_TIME_MS));

        if (Math.abs(Door.OPEN_POS - Door.CLOSED_POS) < 0.02) {
            warn("travel", String.format("open and closed are only %.3f apart - the door barely moves",
                    Math.abs(Door.OPEN_POS - Door.CLOSED_POS)));
        }

        if (!confirm("cycle the door closed - open - closed")) {
            return;
        }

        section("Cycle");
        move(door, "closed", Door.CLOSED_POS);
        move(door, "open", Door.OPEN_POS);
        move(door, "closed again", Door.CLOSED_POS);
    }

    private void move(Servo door, String label, double position) {
        check("door " + label, () -> {
            door.setPosition(position + Door.OFFSET_POS);
            sleep(Math.max(600, Door.ACTUATION_TIME_MS * 3));
            return String.format("commanded %.3f, servo reports %.3f",
                    position + Door.OFFSET_POS, door.getPosition());
        });
    }
}
