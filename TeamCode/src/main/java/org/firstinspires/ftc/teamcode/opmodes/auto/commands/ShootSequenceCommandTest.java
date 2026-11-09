package org.firstinspires.ftc.teamcode.opmodes.auto.commands;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.config.core.MyRobot;
import org.firstinspires.ftc.teamcode.config.core.util.robothelper.OpModeType;
import org.firstinspires.ftc.teamcode.config.core.util.robothelper.SubsystemConfig;
import org.firstinspires.ftc.teamcode.config.subsystems.Turret;

import java.util.List;

/**
 * The whole shot, from a standing start: aim, spin up, fire. This is the one to read if you want to
 * understand how the robot scores.
 *
 * <p><b>What happens on PLAY</b> - two commands, in parallel then in sequence:
 *
 * <ol>
 *   <li>{@code ParallelCommandGroup} of "turret to GOAL" and "shooter active". These are instant and
 *       independent, so they go together rather than one after the other.</li>
 *   <li>{@code r.shootAll()}, which is {@code OuttakeCommandTele} - the real shot sequence the
 *       drivers use. Open that class: it waits for the shooter to be ready, opens the door, runs the
 *       transfer, waits for storage to empty (with a timeout), then puts the intake back.</li>
 * </ol>
 *
 * <p><b>Where the numbers come from</b> - nothing in this file says how fast to spin the flywheel or
 * what angle to point the hood. Every loop, {@code MyRobot.startPeriodic} does this:
 *
 * <pre>
 * ShotCommand cmd = planner.plan(turretPose, vx, vy, omega, goalPose, measuredRpm);
 * turret.input(cmd.predictedTurretPose, cmd.virtualGoal);
 * shooter.setPlannedShot(cmd.distancePoseUnits, cmd.targetRpm, cmd.hoodBaselineDegFromVertical, cmd.possible);
 * </pre>
 *
 * {@code ShotPlanner} works out the distance to the goal from the robot pose, looks the RPM up in a
 * tuned table, solves the hood angle from projectile physics, and reports whether the shot is
 * possible at all. That last flag is what {@code shooter.readyToShoot()} returns, which is what the
 * outtake command waits on. So the chain is: <b>pose -> distance -> RPM + hood -> ready -> fire.</b>
 *
 * <p>This also means the robot has to know where it is. Place it at a sensible distance from the
 * goal and let the pose seed on PLAY, or the planner will solve for the wrong distance.
 *
 * <p><b>If it never fires</b>, the log will show {@code possible = false} - the planner could not
 * find a hood angle for that distance at the table's RPM. There are two known bands where that
 * happens (roughly 50-54 in and 116-126 in); see the unit test
 * {@code ShotPlannerTest.everyDistanceInTheTableHasAReachableHoodAngle}.
 */
@Autonomous(name = "C7 Full shot sequence", group = "Command Test")
public class ShootSequenceCommandTest extends CommandTestOpMode {

    @Override
    protected String subsystemNote() {
        return "whole robot. FLYWHEEL SPINS and the robot will fire whatever is loaded.";
    }

    @Override
    public void initialize() {
        r = new MyRobot(hardwareMap, telemetry, gamepad1, gamepad2,
                List.of(SubsystemConfig.INTAKE, SubsystemConfig.TURRET, SubsystemConfig.SHOOTER,
                        SubsystemConfig.DOOR, SubsystemConfig.FOLLOWER),
                OpModeType.AUTO);
    }

    @Override
    protected Command testSequence() {
        return new SequentialCommandGroup(
                measure("robot pose", () -> r.f.getPose().toString()),
                measure("artifacts loaded", () -> String.valueOf(r.storage.getSize())),

                note("aim + spin up (parallel)"),
                new ParallelCommandGroup(
                        new InstantCommand(() -> r.turret.setTarget(Turret.Target.GOAL)),
                        new InstantCommand(() -> r.shooter.setActive(true))
                ),

                // Give the planner a couple of loops to produce a solution before reading it.
                new WaitCommand(500),
                measure("planner says possible?", () -> String.valueOf(r.shooter.readyToShoot())),
                measure("target rpm", () -> String.format("%.0f", r.shooter.getTargetVelocity())),
                measure("measured rpm", () -> String.format("%.0f", r.shooter.getVelocity())),
                measure("turret target (deg)", () -> String.format("%.1f", Turret.targetTurret)),

                note("shootAll() - the real driver shot sequence"),
                r.shootAll(),

                measure("artifacts left", () -> String.valueOf(r.storage.getSize())),
                measure("door open?", () -> String.valueOf(r.door.isOpen())),

                note("shutting down"),
                new InstantCommand(() -> {
                    r.shooter.setActive(false);
                    r.door.setOpen(false);
                    r.turret.setTarget(Turret.Target.DISABLE);
                }),
                note("done")
        );
    }
}
