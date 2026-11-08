package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
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

@Autonomous(group="Close Auto", name="Close Triple Auto")
public class CloseSoloTripleAuto extends MyCommandOpMode {
    AutoPaths autoPaths;

    @Override
    public void initialize() {
        r = new MyRobot(hardwareMap, telemetry, gamepad1, gamepad2, List.of(SubsystemConfig.INTAKE, SubsystemConfig.TURRET, SubsystemConfig.SHOOTER, SubsystemConfig.DOOR, SubsystemConfig.FOLLOWER), OpModeType.AUTO);
    }

    @Override
    public void atStart() {
        autoPaths = new AutoPaths(r.f, MyRobot.isRed ? Alliance.RED : Alliance.BLUE);
        r.overrideAutoEndPose(autoPaths.getPose(AutoPaths.PoseId.START_FRONT));
        schedule(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new InstantCommand(() -> r.turret.setTarget(Turret.Target.GOAL)),
                                new InstantCommand(() -> r.shooter.setActive(true))
                        ),
                        // Hold the flywheel at each upcoming shot pose's RPM while driving/collecting.
                        r.spinUpShooterFor(autoPaths.getPose(AutoPaths.PoseId.START_FRONT_SHOOT_1)),
                        new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.START_FRONT_TO_SHOOT_FRONT_1)),
                        r.shootAll(),
                        new InstantCommand(() -> r.door.setOpen(false)),

                        new InstantCommand(() -> r.intake.setMode(Intake.Mode.INTAKE)),
                        r.spinUpShooterFor(autoPaths.getPose(AutoPaths.PoseId.START_FRONT_SHOOT_AFTER_FRONT_TRIPLE)),
                        new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.SHOOT_FRONT_1_TO_FRONT_TRIPLE_END)),
                        new WaitCommand(400),
                        new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.FRONT_TRIPLE_END_TO_SHOOT_FRONT)),
                        r.shootAll(),
                        new InstantCommand(() -> r.door.setOpen(false)),

                        new InstantCommand(() -> r.intake.setMode(Intake.Mode.INTAKE)),
                        r.spinUpShooterFor(autoPaths.getPose(AutoPaths.PoseId.SPIKE_MID_THEN_SHOOT_FRONT)),
                        new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.SHOOT_FRONT_TO_MID_TRIPLE_START)),
                        new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.MID_TRIPLE_START_TO_MID_TRIPLE_END)),
                        new WaitCommand(500),
                        new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.MID_TRIPLE_END_TO_GATE_OPEN)),
                        new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.GATE_OPEN_SIDE_TO_FRONT_SHOOT)),
                        r.shootAll(),
                        new InstantCommand(() -> r.door.setOpen(false)),

                        new InstantCommand(() -> r.intake.setMode(Intake.Mode.INTAKE)),
                        r.spinUpShooterFor(autoPaths.getPose(AutoPaths.PoseId.FRONT_SHOOT_AFTER_GATE)),
                        new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.SHOOT_TO_GATE_INTAKE_1)),
                        new WaitCommand(1600),
                        new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.GATE_INTAKE_TO_SHOOT_NORMAL)),
                        r.shootAll(),
                        new InstantCommand(() -> r.door.setOpen(false)),

                        r.spinUpShooterFor(autoPaths.getPose(AutoPaths.PoseId.FRONT_SHOOT_AFTER_GATE)),
                        new RepeatCommand(
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> r.intake.setMode(Intake.Mode.INTAKE)),
                                        new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.SHOOT_TO_GATE_INTAKE_NORMAL)),
                                        new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.TRIPLE_GATE_INTAKE_TO_GATE_INTAKE_SAFE)),
                                        new WaitCommand(1000),
                                        new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.TRIPLE_GATE_INTAKE_SAFE_TO_SHOOT)),
                                        r.shootAll(),
                                        new InstantCommand(() -> r.door.setOpen(false))
                                ),
                                3
                        ),
                        r.clearShooterSpinUp()
                )
        );
    }
}
