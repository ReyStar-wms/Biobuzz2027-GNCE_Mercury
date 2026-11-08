//package org.firstinspires.ftc.teamcode.opmodes.auto;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.seattlesolvers.solverslib.command.InstantCommand;
//import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
//import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
//import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
//import com.seattlesolvers.solverslib.command.WaitCommand;
//import com.seattlesolvers.solverslib.command.WaitUntilCommand;
//
//import org.firstinspires.ftc.teamcode.config.commands.FastFollowPathCommand;
//import org.firstinspires.ftc.teamcode.config.commands.FollowPathCommand;
//import org.firstinspires.ftc.teamcode.config.core.MyCommandOpMode;
//import org.firstinspires.ftc.teamcode.config.core.MyRobot;
//import org.firstinspires.ftc.teamcode.config.core.util.ShotPlanner;
//import org.firstinspires.ftc.teamcode.config.core.util.robothelper.Alliance;
//import org.firstinspires.ftc.teamcode.config.core.util.robothelper.OpModeType;
//import org.firstinspires.ftc.teamcode.config.core.util.robothelper.SubsystemConfig;
//import org.firstinspires.ftc.teamcode.config.paths.AutoPaths2;
//import org.firstinspires.ftc.teamcode.config.subsystems.Intake;
//import org.firstinspires.ftc.teamcode.config.subsystems.Turret;
//
//import java.util.List;
//
//@Autonomous(group="Close Auto", name="Donut 21? partner")
//public class partnerlot extends MyCommandOpMode {
//    AutoPaths2 autoPaths2;
//
//    @Override
//    public void initialize() {
//        r = new MyRobot(hardwareMap, telemetry, gamepad1, gamepad2, List.of(SubsystemConfig.INTAKE, SubsystemConfig.TURRET, SubsystemConfig.SHOOTER, SubsystemConfig.DOOR, SubsystemConfig.FOLLOWER), OpModeType.AUTO);
//    }
//
//    @Override
//    public void atStart() {
//        int WAIT_TIME = 710;
//        autoPaths2 = new AutoPaths2(r.f, MyRobot.isRed ? Alliance.RED : Alliance.BLUE);
//        r.overrideAutoEndPose(autoPaths2.getPose(AutoPaths2.PoseId.FRONT_START));
//        schedule(
//                new SequentialCommandGroup(
//                        // STARTING
//                        // Hold the flywheel at each upcoming shot pose's RPM while driving/collecting.
//                        new ParallelCommandGroup(
//                                new FollowPathCommand(r.f, autoPaths2.getPath(AutoPaths2.PathId.SINGLE_CLOSE_START_TO_MID_SPIKE_START)),
//                                new InstantCommand(() -> r.turret.setTarget(Turret.Target.GOAL)),
//                                new InstantCommand(() -> r.shooter.setActive(true)),
//                                new InstantCommand(() -> r.intake.setMode(Intake.Mode.DISABLE)),
//                                new InstantCommand(() -> r.door.setOpen(true)),
//                                r.spinUpShooterFor(autoPaths2.getPose(AutoPaths2.PoseId.MID_SPIKE_START))
//                        ),
//
//                        new WaitUntilCommand(r.shooter::readyToShoot),
//                        new WaitCommand(180), // These two lines make the hood go to the proper position before actually shooting
//                        r.shootAll2(),
//                        new ParallelCommandGroup(
//                                new InstantCommand(() -> r.door.setOpen(false)),
//
//                                // MID SPIKE CYCLE
//                                new InstantCommand(() -> r.intake.setMode(Intake.Mode.INTAKE)),
//                                r.spinUpShooterFor(autoPaths2.getPose(AutoPaths2.PoseId.FRONT_SHOOT_AFTER_GATE)),
//                                new FollowPathCommand(r.f, autoPaths2.getPath(AutoPaths2.PathId.SINGLE_MID_SPIKE_START_TO_MID_SPIKE_END))
//                        ),
//                        new ParallelCommandGroup(
//                                new InstantCommand(() -> r.intake.setMode(Intake.Mode.DISABLE)),
//                                new InstantCommand(() -> r.door.setOpen(true)),
//                                new FollowPathCommand(r.f, autoPaths2.getPath(AutoPaths2.PathId.SINGLE_MID_SPIKE_END_TO_FRONT_SHOOT_AFTER_GATE))
//                        ),
//                        new WaitCommand(50),
//                        r.shootAll2(),
//
//                        // GATE CYCLE 1
//
//                        new ParallelCommandGroup(
//                                new InstantCommand(() -> r.door.setOpen(false)),
//                                new InstantCommand(() -> r.intake.setMode(Intake.Mode.INTAKE)),
//                                r.spinUpShooterFor(autoPaths2.getPose(AutoPaths2.PoseId.FRONT_SHOOT_AFTER_GATE_NEW)), // TODO: put the initial things in a parallel command
//                                new FastFollowPathCommand(r.f, autoPaths2.getPath(AutoPaths2.PathId.SHOOT_TO_GATE_INTAKE_ONLY_ONE_1))
//                        ),
//                        new ParallelRaceGroup(
//                                new WaitCommand(WAIT_TIME),
//                                new WaitUntilCommand(r.storage::isFull)
//                        ),
//                        new FastFollowPathCommand(r.f, autoPaths2.getPath(AutoPaths2.PathId.GATE_INTAKE_ONLY_ONE_TO_SAFE_1)),
//                        new ParallelCommandGroup(
//                                new InstantCommand(() -> r.intake.setMode(Intake.Mode.DISABLE)),
//                                new InstantCommand(() -> r.door.setOpen(true)),
//                                new FastFollowPathCommand(r.f, autoPaths2.getPath(AutoPaths2.PathId.SAFE_TO_SHOOT_1)),
//                                new InstantCommand(ShotPlanner::enableSOTM),
//                                new InstantCommand(ShotPlanner::enableFPP)
//                        ),
//                        new ParallelCommandGroup(
//                                r.shootAll2(),
//                                new SequentialCommandGroup(
//                                        new FastFollowPathCommand(r.f, autoPaths2.getPath(AutoPaths2.PathId.SHOOT_TO_GATE_INTAKE_ONLY_ONE_2)),
//                                        r.spinUpShooterFor(autoPaths2.getPose(AutoPaths2.PoseId.FRONT_SHOOT_AFTER_GATE_NEW))
//                                )
//                        ),
//                        new ParallelCommandGroup(
//                                new InstantCommand(ShotPlanner::disableSOTM),
//                                new InstantCommand(ShotPlanner::disableFPP),
//                                new InstantCommand(() -> r.door.setOpen(false)),
//                                new InstantCommand(() -> r.intake.setMode(Intake.Mode.INTAKE))
//                        ),
//
//                        // GATE CYCLE 2 (SOTM: shoot while driving to gate 3)
//
//                        new ParallelRaceGroup(
//                                new WaitCommand(WAIT_TIME),
//                                new WaitUntilCommand(r.storage::isFull)
//                        ),
//                        new FastFollowPathCommand(r.f, autoPaths2.getPath(AutoPaths2.PathId.GATE_INTAKE_ONLY_ONE_TO_SAFE_2)),
//                        new ParallelCommandGroup(
//                                new InstantCommand(() -> r.intake.setMode(Intake.Mode.DISABLE)),
//                                new InstantCommand(() -> r.door.setOpen(true)),
//                                new FastFollowPathCommand(r.f, autoPaths2.getPath(AutoPaths2.PathId.SAFE_TO_SHOOT_2)),
//                                new InstantCommand(ShotPlanner::enableSOTM),
//                                new InstantCommand(ShotPlanner::enableFPP)
//                        ),
//                        new ParallelCommandGroup(
//                                r.shootAll2(),
//                                new SequentialCommandGroup(
//                                        new FastFollowPathCommand(r.f, autoPaths2.getPath(AutoPaths2.PathId.SHOOT_TO_GATE_INTAKE_ONLY_ONE_3)),
//                                        r.spinUpShooterFor(autoPaths2.getPose(AutoPaths2.PoseId.FRONT_SHOOT_AFTER_GATE_NEW))
//                                )
//                        ),
//                        new ParallelCommandGroup(
//                                new InstantCommand(ShotPlanner::disableSOTM),
//                                new InstantCommand(ShotPlanner::disableFPP),
//                                new InstantCommand(() -> r.door.setOpen(false)),
//                                new InstantCommand(() -> r.turret.setTarget(Turret.Target.GOAL)),
//                                new InstantCommand(() -> r.intake.setMode(Intake.Mode.INTAKE))
//                        ),
//
//                        // GATE CYCLE 3 (continues from gate 3 position)
//                        new ParallelRaceGroup(
//                                new WaitCommand(WAIT_TIME),
//                                new WaitUntilCommand(r.storage::isFull)
//                        ),
//                        new FastFollowPathCommand(r.f, autoPaths2.getPath(AutoPaths2.PathId.GATE_INTAKE_ONLY_ONE_TO_SAFE_3)),
//                        new ParallelCommandGroup(
//                                new InstantCommand(() -> r.intake.setMode(Intake.Mode.DISABLE)),
//                                new InstantCommand(() -> r.door.setOpen(true)),
//                                new FastFollowPathCommand(r.f, autoPaths2.getPath(AutoPaths2.PathId.SAFE_TO_SHOOT_3)),
//                                new InstantCommand(ShotPlanner::enableSOTM),
//                                new InstantCommand(ShotPlanner::enableFPP)
//                        ),
//                        new ParallelCommandGroup(
//                                r.shootAll2(),
//                                new SequentialCommandGroup(
//                                        new FastFollowPathCommand(r.f, autoPaths2.getPath(AutoPaths2.PathId.SHOOT_TO_GATE_INTAKE_ONLY_ONE_4)),
//                                        r.spinUpShooterFor(autoPaths2.getPose(AutoPaths2.PoseId.FRONT_SHOOT_AFTER_GATE_NEW))
//                                )
//                        ),
//                        new ParallelCommandGroup(
//                                new InstantCommand(ShotPlanner::disableSOTM),
//                                new InstantCommand(ShotPlanner::disableFPP),
//                                new InstantCommand(() -> r.door.setOpen(false)),
//                                new InstantCommand(() -> r.turret.setTarget(Turret.Target.GOAL)),
//                                new InstantCommand(() -> r.intake.setMode(Intake.Mode.INTAKE))
//                        ),
//
//                        // GATE CYCLE 4 (continues from gate 4 position)
//
//                        new ParallelRaceGroup(
//                                new WaitCommand(WAIT_TIME),
//                                new WaitUntilCommand(r.storage::isFull)
//                        ),
//                        new FastFollowPathCommand(r.f, autoPaths2.getPath(AutoPaths2.PathId.GATE_INTAKE_ONLY_ONE_TO_SAFE_4)),
//                        new ParallelCommandGroup(
//                                new InstantCommand(() -> r.intake.setMode(Intake.Mode.DISABLE)),
//                                new InstantCommand(() -> r.door.setOpen(true)),
//                                new FastFollowPathCommand(r.f, autoPaths2.getPath(AutoPaths2.PathId.SAFE_TO_SHOOT_4))
//                        ),
//                        r.shootAll2(),
//                        new ParallelCommandGroup(
//                                new FastFollowPathCommand(r.f, autoPaths2.getPath(AutoPaths2.PathId.SINGLE_SHOOT_AFTER_GATE_TO_CLOSE_SPIKE_END)),
//                                new InstantCommand(() -> r.intake.setMode(Intake.Mode.INTAKE)),
//                                r.spinUpShooterFor(autoPaths2.getPose(AutoPaths2.PoseId.FINAL_SHOOT)),
//                                new InstantCommand(ShotPlanner::disableSOTM),
//                                new InstantCommand(ShotPlanner::disableFPP),
//                                new InstantCommand(() -> r.door.setOpen(false)),
//                                new InstantCommand(() -> r.turret.setTarget(Turret.Target.GOAL))
//                        ),
//
//                        // CLOSE CYCLE (continues from close spike position)
//                        new ParallelCommandGroup(
//                                new InstantCommand(() -> r.intake.setMode(Intake.Mode.DISABLE)),
//                                new InstantCommand(() -> r.door.setOpen(true)),
//                                new FastFollowPathCommand(r.f, autoPaths2.getPath(AutoPaths2.PathId.SINGLE_CLOSE_SPIKE_END_TO_CLOSE_FINAL_SHOOT_BRUNSON))
//                        ),
//                        r.shootAll2(),
//                        new InstantCommand(() -> r.door.setOpen(false)),
//
//
//                        // PARK
//                        new FastFollowPathCommand(r.f, autoPaths2.getPath(AutoPaths2.PathId.BRUNSON_FINAL_PARK)),
//
//
//                        r.clearShooterSpinUp(),
//                        new InstantCommand(() -> r.shooter.setActive(false))
//                )
//        );
//    }
//}