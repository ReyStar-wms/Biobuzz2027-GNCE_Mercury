package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.config.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.config.core.MyCommandOpMode;
import org.firstinspires.ftc.teamcode.config.core.MyRobot;
import org.firstinspires.ftc.teamcode.config.core.util.robothelper.Alliance;
import org.firstinspires.ftc.teamcode.config.core.util.robothelper.OpModeType;
import org.firstinspires.ftc.teamcode.config.core.util.robothelper.SubsystemConfig;
import org.firstinspires.ftc.teamcode.config.paths.AutoPaths2;
import org.firstinspires.ftc.teamcode.config.subsystems.Intake;
import org.firstinspires.ftc.teamcode.config.subsystems.Turret;

import java.util.List;

@Autonomous(group="Close Auto", name="Donut 21? solo")
public class sololot extends MyCommandOpMode {
    AutoPaths2 autoPaths2;

    @Override
    public void initialize() {
        r = new MyRobot(hardwareMap, telemetry, gamepad1, gamepad2, List.of(SubsystemConfig.INTAKE, SubsystemConfig.TURRET, SubsystemConfig.SHOOTER, SubsystemConfig.DOOR, SubsystemConfig.FOLLOWER), OpModeType.AUTO);
    }

    @Override
    public void atStart() {
        autoPaths2 = new AutoPaths2(r.f, MyRobot.isRed ? Alliance.RED : Alliance.BLUE);
        r.overrideAutoEndPose(autoPaths2.getPose(AutoPaths2.PoseId.FRONT_START));
        schedule(
                new SequentialCommandGroup(
                        // STARTING
                        // Hold the flywheel at each upcoming shot pose's RPM while driving/collecting.
                        r.spinUpShooterFor(autoPaths2.getPose(AutoPaths2.PoseId.MID_SPIKE_START)),
                        new ParallelCommandGroup(
                                new FollowPathCommand(r.f, autoPaths2.getPath(AutoPaths2.PathId.SINGLE_CLOSE_START_TO_MID_SPIKE_START)),
                                new InstantCommand(() -> r.turret.setTarget(Turret.Target.GOAL)),
                                new InstantCommand(() -> r.shooter.setActive(true)),
                                new InstantCommand(() -> r.intake.setMode(Intake.Mode.DISABLE)),
                                new InstantCommand(() -> r.door.setOpen(true))
                        ),

                        r.shootAll2(),
                        new InstantCommand(() -> r.door.setOpen(false)),

                        // MID SPIKE CYCLE
                        new InstantCommand(() -> r.intake.setMode(Intake.Mode.INTAKE)),
                        r.spinUpShooterFor(autoPaths2.getPose(AutoPaths2.PoseId.FRONT_SHOOT_AFTER_GATE)),
                        new FollowPathCommand(r.f, autoPaths2.getPath(AutoPaths2.PathId.SINGLE_MID_SPIKE_START_TO_MID_SPIKE_END)),
                        new WaitCommand(25),
                        new InstantCommand(() -> r.intake.setMode(Intake.Mode.DISABLE)),
                        new InstantCommand(() -> r.door.setOpen(true)),
                        new InstantCommand(() -> r.turret.setTarget(Turret.Target.GOAL)),
                        new FollowPathCommand(r.f, autoPaths2.getPath(AutoPaths2.PathId.SINGLE_MID_SPIKE_END_TO_FRONT_SHOOT_AFTER_GATE)),
                        r.shootAll2(),
                        new InstantCommand(() -> r.door.setOpen(false)),

                        // GATE CYCLE 1

                        new InstantCommand(() -> r.intake.setMode(Intake.Mode.INTAKE)),
                        r.spinUpShooterFor(autoPaths2.getPose(AutoPaths2.PoseId.FRONT_SHOOT_AFTER_GATE_NEW)),
                        new FollowPathCommand(r.f, autoPaths2.getPath(AutoPaths2.PathId.SHOOT_TO_GATE_INTAKE_ONLY_ONE_1)),
                        new WaitCommand(850),
                        new InstantCommand(() -> r.intake.setMode(Intake.Mode.DISABLE)),
                        new InstantCommand(() -> r.door.setOpen(true)),

                        new FollowPathCommand(r.f, autoPaths2.getPath(AutoPaths2.PathId.GATE_INTAKE_ONLY_ONE_TO_SHOOT_1)),
                        r.shootAll2(),
                        new InstantCommand(() -> r.door.setOpen(false)),
                        new InstantCommand(() -> r.turret.setTarget(Turret.Target.GOAL)),

                        // GATE CYCLE 2

                        new InstantCommand(() -> r.intake.setMode(Intake.Mode.INTAKE)),
                        r.spinUpShooterFor(autoPaths2.getPose(AutoPaths2.PoseId.FRONT_SHOOT_AFTER_GATE_NEW)),
                        new FollowPathCommand(r.f, autoPaths2.getPath(AutoPaths2.PathId.SHOOT_TO_GATE_INTAKE_ONLY_ONE_2)),
                        new WaitCommand(950),
                        new InstantCommand(() -> r.intake.setMode(Intake.Mode.DISABLE)),
                        new InstantCommand(() -> r.door.setOpen(true)),

                        new FollowPathCommand(r.f, autoPaths2.getPath(AutoPaths2.PathId.GATE_INTAKE_ONLY_ONE_TO_SHOOT_2)),
                        r.shootAll2(),
                        new InstantCommand(() -> r.door.setOpen(false)),
                        new InstantCommand(() -> r.turret.setTarget(Turret.Target.GOAL)),

                        // GATE CYCLE 3

                        new InstantCommand(() -> r.intake.setMode(Intake.Mode.INTAKE)),
                        r.spinUpShooterFor(autoPaths2.getPose(AutoPaths2.PoseId.FRONT_SHOOT_AFTER_GATE_NEW)),
                        new FollowPathCommand(r.f, autoPaths2.getPath(AutoPaths2.PathId.SHOOT_TO_GATE_INTAKE_ONLY_ONE_3)),
                        new WaitCommand(950),
                        new InstantCommand(() -> r.intake.setMode(Intake.Mode.DISABLE)),
                        new InstantCommand(() -> r.door.setOpen(true)),

                        new FollowPathCommand(r.f, autoPaths2.getPath(AutoPaths2.PathId.GATE_INTAKE_ONLY_ONE_TO_SHOOT_PARTNER)),
                        r.shootAll2(),
                        new InstantCommand(() -> r.door.setOpen(false)),
                        new InstantCommand(() -> r.turret.setTarget(Turret.Target.GOAL)),

                        // FAR CYCLE

                        new InstantCommand(() -> r.intake.setMode(Intake.Mode.INTAKE)),
                        r.spinUpShooterFor(autoPaths2.getPose(AutoPaths2.PoseId.FRONT_SHOOT_AFTER_GATE_FINAL_FINAL)),
                        new FollowPathCommand(r.f, autoPaths2.getPath(AutoPaths2.PathId.SINGLE_SHOOT_AFTER_GATE_TO_FAR_SPIKE_END)),
                        new WaitCommand(30),
                        new InstantCommand(() -> r.intake.setMode(Intake.Mode.DISABLE)),
                        new InstantCommand(() -> r.door.setOpen(true)),
                        new InstantCommand(() -> r.turret.setTarget(Turret.Target.GOAL)),
                        new FollowPathCommand(r.f, autoPaths2.getPath(AutoPaths2.PathId.SINGLE_FAR_SPIKE_END_TO_FRONT_SHOOT_FOR_CLOSE_PREP)),
                        r.shootAll2(),
                        new InstantCommand(() -> r.door.setOpen(false)),


                        // CLOSE CYCLE

                        new InstantCommand(() -> r.intake.setMode(Intake.Mode.INTAKE)),
                        r.spinUpShooterFor(autoPaths2.getPose(AutoPaths2.PoseId.FINAL_SHOOT)),
                        new FollowPathCommand(r.f, autoPaths2.getPath(AutoPaths2.PathId.SINGLE_SHOOT_AFTER_GATE_TO_CLOSE_SPIKE_END)),
                        new WaitCommand(25),
                        new InstantCommand(() -> r.intake.setMode(Intake.Mode.DISABLE)),
                        new InstantCommand(() -> r.door.setOpen(true)),
                        new InstantCommand(() -> r.turret.setTarget(Turret.Target.GOAL)),
                        new FollowPathCommand(r.f, autoPaths2.getPath(AutoPaths2.PathId.SINGLE_CLOSE_SPIKE_END_TO_CLOSE_FINAL_SHOOT)),
                        r.shootAll2(),
                        new InstantCommand(() -> r.door.setOpen(false)),
                        r.clearShooterSpinUp(),
                        new InstantCommand(() -> r.shooter.setActive(false))
                )
        );
    }
}