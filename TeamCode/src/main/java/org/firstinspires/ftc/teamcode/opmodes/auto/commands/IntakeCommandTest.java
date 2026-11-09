package org.firstinspires.ftc.teamcode.opmodes.auto.commands;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.config.core.MyRobot;
import org.firstinspires.ftc.teamcode.config.core.util.robothelper.OpModeType;
import org.firstinspires.ftc.teamcode.config.core.util.robothelper.SubsystemConfig;
import org.firstinspires.ftc.teamcode.config.subsystems.Intake;

import java.util.List;

/**
 * The simplest possible command test: run the intake for two seconds, then stop it.
 *
 * <p><b>What to learn here</b> - a subsystem is never told "run for 2 seconds". You set a
 * <em>mode</em> on it and the subsystem's {@code periodic()} keeps acting on that mode every loop
 * until you set a different one. So "run for two seconds" is three commands: set the mode, wait,
 * set it back. The intake's own {@code periodic()} is what actually drives the motors, and the
 * scheduler calls it for you because the intake is a registered subsystem.
 *
 * <p>Note that {@code Intake} has a default command that puts it back into {@code INTAKE} mode
 * whenever nothing else is using it, so {@code DISABLE} only sticks while a command holds the
 * subsystem. That is why the log at the end may show the intake running again.
 */
@Autonomous(name = "C1 Intake command", group = "Command Test")
public class IntakeCommandTest extends CommandTestOpMode {

    @Override
    protected String subsystemNote() {
        return "intake + storage. Rollers will run for 2 s on PLAY.";
    }

    @Override
    public void initialize() {
        // INTAKE also builds Storage - see the MyRobot constructor. Ask for nothing else, so this
        // test still runs on a robot whose turret or shooter is in pieces.
        r = new MyRobot(hardwareMap, telemetry, gamepad1, gamepad2,
                List.of(SubsystemConfig.INTAKE),
                OpModeType.AUTO);
    }

    @Override
    protected Command testSequence() {
        return new SequentialCommandGroup(
                measure("storage count before", () -> String.valueOf(r.storage.getSize())),

                note("intake -> INTAKE"),
                new InstantCommand(() -> r.intake.setMode(Intake.Mode.INTAKE)),
                new WaitCommand(2000),
                measure("intake current (A)", () -> String.format("%.2f", r.intake.getCurrent())),
                measure("intake velocity", () -> String.format("%.0f", r.intake.getIntakeVelocity())),

                note("intake -> DISABLE"),
                new InstantCommand(() -> r.intake.setMode(Intake.Mode.DISABLE)),
                new WaitCommand(500),
                measure("intake current when idle (A)",
                        () -> String.format("%.2f", r.intake.getCurrent())),
                measure("storage count after", () -> String.valueOf(r.storage.getSize())),
                note("done")
        );
    }
}
