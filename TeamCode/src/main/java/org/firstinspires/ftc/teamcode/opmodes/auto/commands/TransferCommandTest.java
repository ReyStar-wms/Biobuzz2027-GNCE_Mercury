package org.firstinspires.ftc.teamcode.opmodes.auto.commands;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.config.commands.TransferCommand;
import org.firstinspires.ftc.teamcode.config.core.MyRobot;
import org.firstinspires.ftc.teamcode.config.core.util.robothelper.OpModeType;
import org.firstinspires.ftc.teamcode.config.core.util.robothelper.SubsystemConfig;

import java.util.List;

/**
 * Runs the existing {@link TransferCommand} - the sequence that pushes whatever is in storage up
 * into the shooter - and logs the storage count on either side of it.
 *
 * <p><b>What to learn here</b> - this is the next step up from {@link IntakeCommandTest}: instead
 * of writing the steps out again, we reuse a command class from {@code config/commands}. Open
 * {@code TransferCommand} and you will see it is exactly the same three ideas as that test
 * (set a mode, wait for a condition, set it back), but written once so every auto can reuse it.
 *
 * <p>Two details in there worth copying into your own commands:
 * <ul>
 *   <li>{@code WaitUntilCommand(() -> storage.getSize() == 0).withTimeout(1000)} - wait for the
 *       real condition, but never longer than a second. Without the timeout, one unreliable beam
 *       break would hang the auto for the rest of the match.</li>
 *   <li>{@code addRequirements(intake)} - declares that this command owns the intake. The scheduler
 *       will cancel anything else that tries to use it at the same time, which is how two commands
 *       are stopped from fighting over one motor.</li>
 * </ul>
 *
 * <p>Load a ball or two before running, otherwise the count is 0 the whole way through and the
 * command just times out - which is itself worth seeing once.
 */
@Autonomous(name = "C2 Transfer command (reuse)", group = "Command Test")
public class TransferCommandTest extends CommandTestOpMode {

    @Override
    protected String subsystemNote() {
        return "intake + storage. Load an artifact first; the transfer will run.";
    }

    @Override
    public void initialize() {
        r = new MyRobot(hardwareMap, telemetry, gamepad1, gamepad2,
                List.of(SubsystemConfig.INTAKE),
                OpModeType.AUTO);
    }

    @Override
    protected Command testSequence() {
        return new SequentialCommandGroup(
                measure("storage count before", () -> String.valueOf(r.storage.getSize())),
                measure("storage full?", () -> String.valueOf(r.storage.isFull())),

                note("running TransferCommand"),
                new TransferCommand(r.intake, r.storage),

                measure("storage count after", () -> String.valueOf(r.storage.getSize())),
                note("done - count should be 0 if anything was transferred")
        );
    }
}
