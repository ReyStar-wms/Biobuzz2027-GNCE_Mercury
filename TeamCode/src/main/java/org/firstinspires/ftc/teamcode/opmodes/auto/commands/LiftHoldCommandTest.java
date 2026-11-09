package org.firstinspires.ftc.teamcode.opmodes.auto.commands;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.config.core.MyRobot;
import org.firstinspires.ftc.teamcode.config.core.util.robothelper.OpModeType;
import org.firstinspires.ftc.teamcode.config.core.util.robothelper.SubsystemConfig;
import org.firstinspires.ftc.teamcode.config.subsystems.Lift;

import java.util.List;

/**
 * Holds the lift at its retracted position and logs the encoder, without climbing.
 *
 * <p><b>What to learn here</b> - the lift has no motors of its own. {@code Lift} drives the four
 * <em>drivetrain</em> motors through a power take-off clutch, so "running the lift" and "driving" are
 * the same four motors in different gear. The PTO servo picks which: {@code CLUTCH_PULLED} for
 * driving, {@code CLUTCH_MESHED} for lifting. That is also why {@code LiftCommand} starts by
 * disabling the intake and the shooter - climbing takes the whole robot.
 *
 * <p>Modes work the way the intake's do: {@code PID} holds {@code Lift.tar} ticks, {@code RAW}
 * applies the power from {@code setManualPower}, {@code INACTIVE} pulls the clutch and stops.
 *
 * <p><b>This test deliberately does not extend the lift.</b> It sets the target to RETRACTED (0
 * ticks) and switches to PID, so the lift holds where it already is and you can watch the encoder,
 * the mode and the clutch respond. To actually climb, run {@code LiftManualControl} on the field
 * with someone spotting, or schedule {@code lift.FullLiftCommand()} / the composite
 * {@code LiftCommand} - both of which drive it to EXTENDED.
 */
@Autonomous(name = "C6 Lift command (hold, no climb)", group = "Command Test")
public class LiftHoldCommandTest extends CommandTestOpMode {

    @Override
    protected String subsystemNote() {
        return "lift only. Holds at retracted - it will NOT climb. The clutch will shift.";
    }

    @Override
    public void initialize() {
        r = new MyRobot(hardwareMap, telemetry, gamepad1, gamepad2,
                List.of(SubsystemConfig.LIFT),
                OpModeType.AUTO);
    }

    @Override
    protected Command testSequence() {
        return new SequentialCommandGroup(
                measure("encoder at start", () -> String.valueOf(r.lift.getCurrentPosition())),
                measure("mode at start", () -> String.valueOf(r.lift.getMode())),

                note("target RETRACTED, mode -> PID (clutch meshes, lift holds 0)"),
                new InstantCommand(() -> r.lift.setTargetPosition(Lift.LiftPositions.RETRACTED)),
                r.lift.SetModeCommand(Lift.Mode.PID),
                new WaitCommand(1500),
                measure("target ticks", () -> String.valueOf(Lift.tar)),
                measure("encoder", () -> String.valueOf(r.lift.getCurrentPosition())),
                measure("reachedTarget()", () -> String.valueOf(r.lift.reachedTarget())),
                measure("mode", () -> String.valueOf(r.lift.getMode())),

                note("mode -> INACTIVE (clutch pulls, wheels drive again)"),
                r.lift.SetModeCommand(Lift.Mode.INACTIVE),
                new WaitCommand(1000),
                measure("mode", () -> String.valueOf(r.lift.getMode())),
                measure("encoder", () -> String.valueOf(r.lift.getCurrentPosition())),
                note("done - always leave the lift INACTIVE so the drivetrain works")
        );
    }
}
