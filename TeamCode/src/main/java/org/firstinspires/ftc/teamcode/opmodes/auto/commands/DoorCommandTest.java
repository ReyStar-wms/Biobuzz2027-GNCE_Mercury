package org.firstinspires.ftc.teamcode.opmodes.auto.commands;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.config.core.MyRobot;
import org.firstinspires.ftc.teamcode.config.core.util.robothelper.OpModeType;
import org.firstinspires.ftc.teamcode.config.core.util.robothelper.SubsystemConfig;
import org.firstinspires.ftc.teamcode.config.subsystems.Door;

import java.util.List;

/**
 * Opens and closes the shooter door using the subsystem's own command factory.
 *
 * <p><b>What to learn here</b> - a subsystem can hand out commands as well as plain setters.
 * {@code door.setOpenCommand(true)} returns a command that sets the state <em>and</em> waits
 * {@code Door.ACTUATION_TIME_MS} for the servo to physically get there, so the next command in your
 * sequence does not start while the door is still moving. Compare that with calling
 * {@code door.setOpen(true)} yourself, which returns instantly and leaves the waiting to you.
 *
 * <p>Prefer the command form inside a sequence; the plain setter is for teleop, where the next loop
 * is coming anyway.
 *
 * <p><b>Also worth knowing</b> - the door has a default command (set up in its constructor) that
 * opens it automatically once storage holds three artifacts, and closes it otherwise. A default
 * command runs whenever nothing else has claimed the subsystem, which is how the door behaves
 * sensibly in teleop with nobody driving it. It is also why the door may move again on its own
 * after this test finishes.
 */
@Autonomous(name = "C5 Door command", group = "Command Test")
public class DoorCommandTest extends CommandTestOpMode {

    @Override
    protected String subsystemNote() {
        return "door only. It will cycle open and closed. Keep fingers out of the gate.";
    }

    @Override
    public void initialize() {
        r = new MyRobot(hardwareMap, telemetry, gamepad1, gamepad2,
                List.of(SubsystemConfig.DOOR),
                OpModeType.AUTO);
    }

    @Override
    protected Command testSequence() {
        return new SequentialCommandGroup(
                measure("positions", () -> String.format("open %.3f, closed %.3f, %d ms travel",
                        Door.OPEN_POS, Door.CLOSED_POS, Door.ACTUATION_TIME_MS)),
                measure("isOpen() at start", () -> String.valueOf(r.door.isOpen())),

                note("setOpenCommand(true)"),
                r.door.setOpenCommand(true),
                measure("isOpen()", () -> String.valueOf(r.door.isOpen())),
                new WaitCommand(1000),

                note("setOpenCommand(false)"),
                r.door.setOpenCommand(false),
                measure("isOpen()", () -> String.valueOf(r.door.isOpen())),
                note("done")
        );
    }
}
