package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
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
import org.firstinspires.ftc.teamcode.config.subsystems.Turret;

import java.util.List;

@Autonomous(group="Far Auto", name="Far + Far Spike Opt Auto SL")
public class FarOptSLWithFarSpikeAuto extends MyCommandOpMode {
    AutoPaths autoPaths;

    @Override
    public void initialize() {
        r = new MyRobot(hardwareMap, telemetry, gamepad1, gamepad2, List.of(SubsystemConfig.INTAKE, SubsystemConfig.TURRET, SubsystemConfig.SHOOTER, SubsystemConfig.DOOR, SubsystemConfig.FOLLOWER), OpModeType.AUTO);
    }

    @Override
    public void atStart() {
        autoPaths = new AutoPaths(r.f, MyRobot.isRed ? Alliance.RED : Alliance.BLUE);
        // INITIAL 3
        r.overrideAutoEndPose(autoPaths.getPose(AutoPaths.PoseId.START_BACK));
        schedule(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new InstantCommand(() -> r.turret.setTarget(Turret.Target.GOAL)),
                                new InstantCommand(() -> r.shooter.setActive(true))
                        ),
                        new WaitCommand(250),
                        r.shootAll(),
                        new InstantCommand(() -> r.door.setOpen(false)),
                        new InstantCommand(() -> r.intake.setMode(Intake.Mode.INTAKE)),

                        // Hold the flywheel at the next shot pose's RPM while collecting near the goal.
                        r.spinUpShooterFor(autoPaths.getPose(AutoPaths.PoseId.SHOOT_BACK_1)),
                        // HUMAN PLAYER CYCLE
                        new ParallelRaceGroup(
                                new SequentialCommandGroup(
                                        new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.START_BACK_TO_HUMAN_PLAYER_END)),
                                        new WaitCommand(500)
                                ),
                                new WaitUntilCommand(() -> r.storage.getSize() == 3)
                        ),
                        r.goToLinear(autoPaths.getPose(AutoPaths.PoseId.SHOOT_BACK_1)),
                        new WaitCommand(125),
                        r.shootAll(),
                        new InstantCommand(() -> r.door.setOpen(false)),
                        new InstantCommand(() -> r.intake.setMode(Intake.Mode.INTAKE)),

                        // FAR SPIKE CYCLE
                        r.spinUpShooterFor(autoPaths.getPose(AutoPaths.PoseId.SHOOT_BACK_HP_PREP)),
                        new ParallelRaceGroup(
                                new SequentialCommandGroup(
                                        new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.SHOOT_BACK_1_TO_FAR_SPIKE_END)),
                                        new WaitCommand(300)
                                ),
                                new WaitUntilCommand(() -> r.storage.getSize() == 3)
                        ),
                        r.goToLinear(autoPaths.getPose(AutoPaths.PoseId.SHOOT_BACK_HP_PREP)),
                        new WaitCommand(100),
                        r.shootAll(),
                        new InstantCommand(() -> r.door.setOpen(false)),
                        new InstantCommand(() -> r.intake.setMode(Intake.Mode.INTAKE)),

                        // STRAIGHT CYCLE 1
                        r.spinUpShooterFor(autoPaths.getPose(AutoPaths.PoseId.SHOOT_BACK_GATHER_PREP)),
                        new ParallelRaceGroup(
                                new SequentialCommandGroup(
                                        r.goToLinear(autoPaths.getPose(AutoPaths.PoseId.HP_END)),
                                        new WaitCommand(300)
                                ),
                                new WaitUntilCommand(() -> r.storage.getSize() == 3)
                        ),
                        r.goToLinear(autoPaths.getPose(AutoPaths.PoseId.SHOOT_BACK_GATHER_PREP)),
                        new WaitCommand(100),
                        r.shootAll(),
                        new InstantCommand(() -> r.door.setOpen(false)),
                        new InstantCommand(() -> r.intake.setMode(Intake.Mode.INTAKE)),

                        // CURVE CYCLE 1
                        r.spinUpShooterFor(autoPaths.getPose(AutoPaths.PoseId.SHOOT_BACK_HP_PREP)),
                        new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.GATHER_PREP_TO_GATHER_END)),
                        r.goToLinear(autoPaths.getPose(AutoPaths.PoseId.SHOOT_BACK_HP_PREP)),
                        new WaitCommand(100),
                        r.shootAll(),
                        new InstantCommand(() -> r.door.setOpen(false)),
                        new InstantCommand(() -> r.intake.setMode(Intake.Mode.INTAKE)),

                        // STRAIGHT CYCLE 2
                        r.spinUpShooterFor(autoPaths.getPose(AutoPaths.PoseId.SHOOT_BACK_HP_PREP)),
                        new RepeatCommand(
                                new SequentialCommandGroup(
                                        new ParallelRaceGroup(
                                                new SequentialCommandGroup(
                                                        r.goToLinear(autoPaths.getPose(AutoPaths.PoseId.HP_END)),
                                                        new WaitCommand(300)
                                                ),
                                                new WaitUntilCommand(() -> r.storage.getSize() == 3)
                                        ),
                                        r.goToLinear(autoPaths.getPose(AutoPaths.PoseId.SHOOT_BACK_HP_PREP)),
                                        r.shootAll(),
                                        new InstantCommand(() -> r.door.setOpen(false)),
                                        new InstantCommand(() -> r.intake.setMode(Intake.Mode.INTAKE))
                                ),
                                1
                        ),

                        // CURVE CYCLE 2
                        r.spinUpShooterFor(autoPaths.getPose(AutoPaths.PoseId.SHOOT_BACK_HP_PREP)),
                        new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.GATHER_PREP_TO_GATHER_END_SL)),
                        r.goToLinear(autoPaths.getPose(AutoPaths.PoseId.SHOOT_BACK_HP_PREP)),
                        new WaitCommand(100),
                        r.shootAll(),
                        new InstantCommand(() -> r.door.setOpen(false)),
                        new InstantCommand(() -> r.intake.setMode(Intake.Mode.INTAKE))
                )
                        .raceWith(new WaitCommand(29500))
                        .andThen(r.clearShooterSpinUp())
                        .andThen(r.goToLinear(autoPaths.getPose(AutoPaths.PoseId.PARK_FINAL)))
        );
    }
}
