//package org.firstinspires.ftc.teamcode.opmodes.auto;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.seattlesolvers.solverslib.command.InstantCommand;
//import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
//import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
//import com.seattlesolvers.solverslib.command.WaitCommand;
//
//import org.firstinspires.ftc.teamcode.config.commands.FollowPathCommand;
//import org.firstinspires.ftc.teamcode.config.core.MyCommandOpMode;
//import org.firstinspires.ftc.teamcode.config.core.MyRobot;
//import org.firstinspires.ftc.teamcode.config.core.util.robothelper.Alliance;
//import org.firstinspires.ftc.teamcode.config.core.util.robothelper.OpModeType;
//import org.firstinspires.ftc.teamcode.config.core.util.robothelper.SubsystemConfig;
//import org.firstinspires.ftc.teamcode.config.paths.AutoPaths;
//import org.firstinspires.ftc.teamcode.config.subsystems.Turret;
//
//import java.util.List;
//
//@Autonomous(group="Partner Auto", name="Front Double Gate Auto")
//public class FrontDoubleGateAuto extends MyCommandOpMode {
//    AutoPaths autoPaths;
//
//    @Override
//    public void initialize() {
//        r = new MyRobot(hardwareMap, telemetry, gamepad1, gamepad2, List.of(SubsystemConfig.INTAKE, SubsystemConfig.TURRET, SubsystemConfig.SHOOTER, SubsystemConfig.DOOR, SubsystemConfig.FOLLOWER), OpModeType.AUTO);
//    }
//
//    @Override
//    public void initialize_loop() {
//    }
//
//    @Override
//    public void atStart() {
//        autoPaths = new AutoPaths(r.f, MyRobot.isRed ? Alliance.RED : Alliance.BLUE);
//        r.overrideAutoEndPose(autoPaths.getPose(AutoPaths.PoseId.START_FRONT));
//        schedule(
//                new SequentialCommandGroup(
//                        new ParallelCommandGroup(
//                                new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.START_FRONT_TO_SHOOT_FRONT)),
//                                new InstantCommand(() -> r.turret.setTarget(Turret.Target.GOAL)),
//                                new InstantCommand(() -> r.shooter.setActive(true))
//                        ),
//                        r.shootAll(),
//                        new InstantCommand(() -> r.door.setOpen(false)),
//                        new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.SHOOT_FRONT_TO_FRONT_SPIKE_START)),
//                        new SequentialCommandGroup(
//                                new InstantCommand(() -> r.f.setMaxPower(0.5)),
//                                new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.FRONT_SPIKE_START_TO_FRONT_SPIKE_END)),
//                                new InstantCommand(() -> r.f.setMaxPower(1))
//                        ),
//                        new InstantCommand(() -> r.f.setMaxPower(0.8)),
//                        new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.FRONT_SPIKE_END_TO_GATE)),
//                        new WaitCommand(5000),
//                        r.goTo(autoPaths.getPose(AutoPaths.PoseId.SHOOT_FRONT)),
//                        r.shootAll(),
//                        new InstantCommand(() -> r.door.setOpen(false)),
//                        new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.SHOOT_FRONT_TO_MID_SPIKE_START)),
//                        new SequentialCommandGroup(
//                                new InstantCommand(() -> r.f.setMaxPower(0.5)),
//                                new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.MID_SPIKE_START_TO_MID_SPIKE_END)),
//                                new InstantCommand(() -> r.f.setMaxPower(1))
//                        ),
//                        new InstantCommand(() -> r.f.setMaxPower(1)),
//                        new FollowPathCommand(r.f, autoPaths.getPath(AutoPaths.PathId.MID_SPIKE_END_TO_SHOOT_FRONT)),
//                        r.shootAll(),
//                        new InstantCommand(() -> r.door.setOpen(false))
//                ).withTimeout(29500).andThen(r.goTo(autoPaths.getPose(AutoPaths.PoseId.FRONT_PARK)))
//        );
//    }
//}
