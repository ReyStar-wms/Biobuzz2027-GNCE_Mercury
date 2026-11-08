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

@Autonomous(group="Close Auto", name="Close SINGLE Auto")
public class CloseSoloAuto extends MyCommandOpMode {
    AutoPaths autoPaths;

    @Override
    public void initialize() {
        r = new MyRobot(hardwareMap, telemetry, gamepad1, gamepad2, List.of(SubsystemConfig.INTAKE, SubsystemConfig.TURRET, SubsystemConfig.SHOOTER, SubsystemConfig.DOOR, SubsystemConfig.FOLLOWER), OpModeType.AUTO);
    }

    @Override
    public void atStart() {
        autoPaths = new AutoPaths(r.f, MyRobot.isRed ? Alliance.RED : Alliance.BLUE);
        r.overrideAutoEndPose(autoPaths.getPose(AutoPaths.PoseId.SINGLE_FRONT_START));
        schedule(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new InstantCommand(() -> r.turret.setTarget(Turret.Target.GOAL)),
                                new InstantCommand(() -> r.shooter.setActive(true))
                        ),
                        // Hold the flywheel at each upcoming shot pose's RPM while driving/collecting.
                        r.spinUpShooterFor(autoPaths.getPose(AutoPaths.PoseId.SINGLE_MID_SPIKE_START)),
                        new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.SINGLE_CLOSE_START_TO_MID_SPIKE_START)),
                        r.shootAll(),
                        new InstantCommand(() -> r.door.setOpen(false)),

                        new InstantCommand(() -> r.intake.setMode(Intake.Mode.INTAKE)),
                        r.spinUpShooterFor(autoPaths.getPose(AutoPaths.PoseId.FRONT_SHOOT_AFTER_GATE)),
                        new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.SINGLE_MID_SPIKE_START_TO_MID_SPIKE_END)),
                        new WaitCommand(100),
                        new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.SINGLE_MID_SPIKE_END_TO_FRONT_SHOOT_AFTER_GATE)),
                        r.shootAll(),
                        new InstantCommand(() -> r.door.setOpen(false)),

                        r.spinUpShooterFor(autoPaths.getPose(AutoPaths.PoseId.FRONT_SHOOT_AFTER_GATE)),
                        new RepeatCommand(
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> r.intake.setMode(Intake.Mode.INTAKE)),
                                        new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.SHOOT_TO_GATE_INTAKE_NORMAL)),
                                        new WaitCommand(400),
                                        new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.TRIPLE_GATE_INTAKE_TO_GATE_INTAKE_SAFE)),
                                        new WaitCommand(800),
                                        new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.TRIPLE_GATE_INTAKE_SAFE_TO_SHOOT)),
                                        r.shootAll(),
                                        new InstantCommand(() -> r.door.setOpen(false))
                                ),
                                2
                        ),

                        r.spinUpShooterFor(autoPaths.getPose(AutoPaths.PoseId.SINGLE_FINAL_SHOOT)),
                        new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.SINGLE_SHOOT_AFTER_GATE_TO_CLOSE_SPIKE_START)),
                        new InstantCommand(() -> r.intake.setMode(Intake.Mode.INTAKE)),
                        new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.SINGLE_CLOSE_SPIKE_START_TO_CLOSE_SPIKE_END)),
                        new WaitCommand(100),
                        new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.SINGLE_CLOSE_SPIKE_END_TO_CLOSE_FINAL_SHOOT)),
                        r.shootAll(),
                        new InstantCommand(() -> r.door.setOpen(false)),
                        r.clearShooterSpinUp()
                )
        );
    }
}
