package org.firstinspires.ftc.teamcode.opmodes.auto.commands;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.config.core.MyRobot;
import org.firstinspires.ftc.teamcode.config.core.util.robothelper.OpModeType;
import org.firstinspires.ftc.teamcode.config.core.util.robothelper.SubsystemConfig;
import org.firstinspires.ftc.teamcode.config.subsystems.Shooter;

import java.util.List;

/**
 * Spins the flywheel up to a fixed RPM, waits for it to get there, and logs how long it took.
 *
 * <p><b>What to learn here</b> - this is the first test that waits on a <em>condition</em> rather
 * than a fixed time, which is the pattern most of the autos are built from:
 *
 * <pre>new WaitUntilCommand(() -&gt; condition).withTimeout(6000)</pre>
 *
 * The lambda is re-evaluated every loop until it returns true. Always pair it with a timeout, so a
 * flywheel that never reaches speed costs you six seconds instead of the match.
 *
 * <p><b>Two things about the shooter that catch people out:</b>
 *
 * <ol>
 *   <li>The flywheel's target normally comes from {@code ShotPlanner} via
 *       {@code setPlannedShot(...)}, which {@code MyRobot.startPeriodic} only calls when the
 *       FOLLOWER is built - it needs a robot pose to work out the distance to the goal. This test
 *       asks for the shooter alone, so instead it uses {@code setSpinUpRpm(...)}, the override that
 *       pins the target to one RPM. That override is what the autos use to hold shot speed while
 *       driving in to collect.</li>
 *   <li>For the same reason {@code readyToShoot()} is false throughout this test: it reports
 *       whether the <em>planner</em> declared a shot possible, not whether the wheel is at speed.
 *       So this test compares the measured velocity against the target itself. See
 *       {@link ShootSequenceCommandTest} for the full version with the planner running.</li>
 * </ol>
 */
@Autonomous(name = "C3 Shooter command (wait until)", group = "Command Test")
public class ShooterCommandTest extends CommandTestOpMode {

    private static final double TEST_RPM = 1200;

    @Override
    protected String subsystemNote() {
        return "shooter only. THE FLYWHEEL WILL SPIN to " + (int) TEST_RPM + " rpm - stand clear.";
    }

    @Override
    public void initialize() {
        r = new MyRobot(hardwareMap, telemetry, gamepad1, gamepad2,
                List.of(SubsystemConfig.SHOOTER),
                OpModeType.AUTO);
    }

    @Override
    protected Command testSequence() {
        return new SequentialCommandGroup(
                measure("velocity at rest", () -> String.format("%.0f", r.shooter.getVelocity())),

                note("setActive(true) + setSpinUpRpm(" + (int) TEST_RPM + ")"),
                new InstantCommand(() -> {
                    r.shooter.setActive(true);
                    r.shooter.setSpinUpRpm(TEST_RPM);
                }),
                measure("spin-up held?", () -> String.valueOf(r.shooter.isSpinUpHeld())),

                note("waiting for the wheel to reach target"),
                new WaitUntilCommand(this::atSpeed).withTimeout(6000),
                measure("velocity", () -> String.format("%.0f", r.shooter.getVelocity())),
                measure("target", () -> String.format("%.0f", r.shooter.getTargetVelocity())),
                measure("at speed?", () -> String.valueOf(atSpeed())),
                measure("readyToShoot() (needs the planner)",
                        () -> String.valueOf(r.shooter.readyToShoot())),

                note("holding for 1 s to see if it stays there"),
                new WaitCommand(1000),
                measure("velocity after hold", () -> String.format("%.0f", r.shooter.getVelocity())),

                note("clearSpinUp() + setActive(false)"),
                new InstantCommand(() -> {
                    r.shooter.clearSpinUp();
                    r.shooter.setActive(false);
                }),
                new WaitCommand(2000),
                measure("velocity after coasting 2 s",
                        () -> String.format("%.0f", r.shooter.getVelocity())),
                note("done")
        );
    }

    /** The shooter's own readiness window, applied to the measured velocity. */
    private boolean atSpeed() {
        return Math.abs(r.shooter.getTargetVelocity() - r.shooter.getVelocity())
                < Shooter.VELOCITY_READY_THRESHOLD;
    }
}
