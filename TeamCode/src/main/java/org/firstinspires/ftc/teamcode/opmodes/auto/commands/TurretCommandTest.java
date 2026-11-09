package org.firstinspires.ftc.teamcode.opmodes.auto.commands;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.config.core.MyRobot;
import org.firstinspires.ftc.teamcode.config.core.util.robothelper.OpModeType;
import org.firstinspires.ftc.teamcode.config.core.util.robothelper.SubsystemConfig;
import org.firstinspires.ftc.teamcode.config.subsystems.Turret;

import java.util.List;

/**
 * Points the turret at the goal, then disables it, logging the angle it chose each time.
 *
 * <p><b>What to learn here</b> - the turret does not take an angle. You give it a <em>target kind</em>
 * ({@code GOAL}, {@code MOTIF}, {@code DISABLE}) and every loop it recomputes the angle from the
 * robot's current pose and the goal's field position. Drive the robot and the turret follows without
 * anyone commanding a new number. That is the whole idea behind {@code Turret.Target}.
 *
 * <p><b>Why this test asks for three subsystems</b> - the chain that aims the turret is longer than
 * it looks. {@code MyRobot.startPeriodic} only calls {@code turret.input(robotPose, goalPose)}
 * inside its SHOOTER branch, and that branch only runs when the FOLLOWER exists, because the aim is
 * computed from the robot pose. So TURRET on its own would build and then never be told where to
 * look. When you add a subsystem to a test, check what feeds it in {@code startPeriodic}.
 *
 * <p>The goal pose itself is picked by alliance: {@code MyRobot} mirrors the blue goal across the
 * field for red, so this test's numbers depend on the alliance chosen during init.
 */
@Autonomous(name = "C4 Turret command (aim at goal)", group = "Command Test")
public class TurretCommandTest extends CommandTestOpMode {

    @Override
    protected String subsystemNote() {
        return "follower + turret + shooter. The turret will swing to face the goal.";
    }

    @Override
    public void initialize() {
        r = new MyRobot(hardwareMap, telemetry, gamepad1, gamepad2,
                List.of(SubsystemConfig.FOLLOWER, SubsystemConfig.TURRET, SubsystemConfig.SHOOTER),
                OpModeType.AUTO);
    }

    @Override
    protected Command testSequence() {
        return new SequentialCommandGroup(
                // The shooter is built only so the aiming branch runs; keep the flywheel off.
                new InstantCommand(() -> r.shooter.setActive(false)),
                measure("robot pose", () -> r.f.getPose().toString()),
                measure("alliance", () -> MyRobot.isRed ? "RED" : "BLUE"),
                // The goal MyRobot actually aims at is this blue pose, mirrored across the field
                // for red. The field is 141.5 in wide here, which is also Pedro's mirror axis.
                measure("blue goal", () -> String.format("(%.1f, %.1f)",
                        MyRobot.blueGoalPoseX, MyRobot.blueGoalPoseY)),

                note("turret -> GOAL"),
                new InstantCommand(() -> r.turret.setTarget(Turret.Target.GOAL)),
                new WaitCommand(1500),
                measure("target angle (deg)", () -> String.format("%.1f", Turret.targetTurret)),
                measure("wrap count", () -> String.valueOf(r.turret.getWrapCount())),
                measure("reachedTarget()", () -> String.valueOf(r.turret.reachedTarget())),

                // reachedTarget() is pinned true by the alwaysAtTarget flag the constructor sets,
                // so a WaitUntilCommand on it returns immediately - which is why the turret wait in
                // OuttakeCommand is commented out. Turn the flag off to see the real comparison.
                note("setAlwaysAtTarget(false) - now it reports the real check"),
                new InstantCommand(() -> r.turret.setAlwaysAtTarget(false)),
                new WaitCommand(500),
                measure("reachedTarget() for real", () -> String.valueOf(r.turret.reachedTarget())),

                note("turret -> DISABLE"),
                new InstantCommand(() -> r.turret.setTarget(Turret.Target.DISABLE)),
                new WaitCommand(1500),
                measure("target angle after disable",
                        () -> String.format("%.1f", Turret.targetTurret)),
                note("done - DISABLE parks the target at 0, it does not cut the servos")
        );
    }
}
