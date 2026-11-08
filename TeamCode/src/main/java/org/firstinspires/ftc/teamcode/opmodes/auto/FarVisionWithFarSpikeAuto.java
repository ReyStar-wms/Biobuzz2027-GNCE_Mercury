package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.RepeatCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.config.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.config.core.MyCommandOpMode;
import org.firstinspires.ftc.teamcode.config.core.MyRobot;
import org.firstinspires.ftc.teamcode.config.core.util.robothelper.Alliance;
import org.firstinspires.ftc.teamcode.config.core.util.robothelper.OpModeType;
import org.firstinspires.ftc.teamcode.config.core.util.robothelper.SubsystemConfig;
import org.firstinspires.ftc.teamcode.config.paths.AutoPaths;
import org.firstinspires.ftc.teamcode.config.subsystems.Intake;
import org.firstinspires.ftc.teamcode.config.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.config.subsystems.Turret;

import java.util.List;

@Autonomous(group="Far Auto", name="Far Vision + Far Spike Auto")
public class FarVisionWithFarSpikeAuto extends MyCommandOpMode {
    AutoPaths autoPaths;

    // Soft field-y guard. We never want the robot to wander far past this line during a collect
    // (e.g. chasing a ball or a path going long). Instead of a hard abort, crossing MAX_Y now just
    // cuts the current collect short -- the robot drives back to its shoot pose, fires whatever it
    // has, and the cycle continues. Because this fires AFTER y crosses the line and braking/turning
    // isn't instant, the robot will overshoot slightly before goToLinear curls it back, so set this
    // a touch below the true hard limit if you must guarantee y never physically passes it.
    // NOTE: this guard is only active during collect phases. The shoot poses below are assumed to be
    // safely under MAX_Y (goToLinear drives straight to them, bringing y back under the line).
    public static double MAX_Y = 36;

    @Override
    public void initialize() {
        r = new MyRobot(hardwareMap, telemetry, gamepad1, gamepad2, List.of(SubsystemConfig.INTAKE, SubsystemConfig.TURRET, SubsystemConfig.SHOOTER, SubsystemConfig.DOOR, SubsystemConfig.FOLLOWER, SubsystemConfig.LL), OpModeType.AUTO);
    }

    @Override
    public void atStart() {
        autoPaths = new AutoPaths(r.f, MyRobot.isRed ? Alliance.RED : Alliance.BLUE);
        r.overrideAutoEndPose(autoPaths.getPose(AutoPaths.PoseId.START_BACK));
        schedule(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new InstantCommand(() -> r.turret.setTarget(Turret.Target.GOAL)),
                                new InstantCommand(() -> r.shooter.setActive(true)),
                                new InstantCommand(() -> r.ll.setMode(Limelight.Mode.BALL_DETECTION))
                        ),
                        new WaitCommand(250),
                        r.shootAll(),
                        new InstantCommand(() -> r.door.setOpen(false)),
                        new InstantCommand(() -> r.intake.setMode(Intake.Mode.INTAKE)),

                        // Keep the flywheel spun for the next shot pose while driving in to collect.
                        r.spinUpShooterFor(autoPaths.getPose(AutoPaths.PoseId.SHOOT_BACK_1)),
                        new ParallelRaceGroup(
                                new SequentialCommandGroup(
                                        new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.START_BACK_TO_HUMAN_PLAYER_END)),
                                        new WaitCommand(500)
                                ),
                                new WaitUntilCommand(() -> r.storage.getSize() == 3),
                                // Y-LIMIT RECOVERY: bail out of the collect early if we drift over MAX_Y,
                                // then fall through to the shoot pose + shootAll below.
                                new WaitUntilCommand(() -> r.f.getPose().getY() > MAX_Y)
                        ),
                        r.goToLinear(autoPaths.getPose(AutoPaths.PoseId.SHOOT_BACK_1)),
                        new WaitCommand(100),
                        r.shootAll(),
                        new InstantCommand(() -> r.door.setOpen(false)),
                        new InstantCommand(() -> r.intake.setMode(Intake.Mode.INTAKE)),

                        r.spinUpShooterFor(autoPaths.getPose(AutoPaths.PoseId.SHOOT_BACK_3)),
                        new ParallelRaceGroup(
                                new SequentialCommandGroup(
                                        new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.SHOOT_BACK_1_TO_FAR_SPIKE_END)),
                                        new WaitCommand(300)
                                ),
                                new WaitUntilCommand(() -> r.storage.getSize() == 3),
                                new WaitUntilCommand(() -> r.f.getPose().getY() > MAX_Y)
                        ),
                        r.goToLinear(autoPaths.getPose(AutoPaths.PoseId.SHOOT_BACK_3)),
                        // Scan for balls during this shot so the collect that follows needs no scan wait.
                        new ParallelCommandGroup(
                                r.shootAll(),
                                r.scanForBalls()
                        ),
                        new InstantCommand(() -> r.door.setOpen(false)),
                        new InstantCommand(() -> r.intake.setMode(Intake.Mode.INTAKE)),

                        // Every loop shoots + scans from SHOOT_BACK_SCAN, so hold the flywheel there throughout.
                        r.spinUpShooterFor(autoPaths.getPose(AutoPaths.PoseId.SHOOT_BACK_SCAN)),
                        new RepeatCommand(
                                new SequentialCommandGroup(
                                        new ParallelRaceGroup(
                                                r.collectThreeBallsNoScan(),
                                                new WaitUntilCommand(() -> r.storage.getSize() == 3),
                                                // Y-LIMIT RECOVERY: same as above -- crossing MAX_Y ends the
                                                // collect, then we go back to SHOOT_BACK_SCAN, shoot what we
                                                // have, and the RepeatCommand carries on with the next loop.
                                                new WaitUntilCommand(() -> r.f.getPose().getY() > MAX_Y)
                                        ),
                                        r.goToLinear(autoPaths.getPose(AutoPaths.PoseId.SHOOT_BACK_SCAN)),
                                        // Shoot and re-scan at the same time, so the next loop's collect is ready immediately.
                                        new ParallelCommandGroup(
                                                r.shootAll(),
                                                r.scanForBalls()
                                        ),
                                        new InstantCommand(() -> r.door.setOpen(false)),
                                        new InstantCommand(() -> r.intake.setMode(Intake.Mode.INTAKE))
                                ),
                                6
                        )
                )
                        .raceWith(new WaitCommand(29500))
                        .andThen(r.clearShooterSpinUp())
                        .andThen(r.goToLinear(autoPaths.getPose(AutoPaths.PoseId.PARK_FINAL)))
        );
    }
}