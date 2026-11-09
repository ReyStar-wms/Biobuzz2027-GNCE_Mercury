package org.firstinspires.ftc.teamcode.opmodes.auto.commands;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.config.core.MyRobot;
import org.firstinspires.ftc.teamcode.config.core.util.robothelper.OpModeType;
import org.firstinspires.ftc.teamcode.config.core.util.robothelper.SubsystemConfig;

import java.util.List;

/**
 * Drives 12 inches forward along a Pedro path, then logs how close it landed.
 *
 * <p><b>What to learn here</b> - following a path is two commands, and this is the pattern every
 * auto is built from:
 *
 * <pre>
 * new InstantCommand(() -&gt; f.followPath(path))   // hand the path over, return immediately
 * new WaitUntilCommand(() -&gt; !f.isBusy())        // then wait for the follower to finish
 * </pre>
 *
 * That pair already exists as {@link org.firstinspires.ftc.teamcode.config.commands.FollowPathCommand},
 * and {@code MyRobot.goToLinear(pose)} wraps it for a straight line to a pose. This test writes it out
 * longhand so you can see there is nothing else to it. Nobody waits in a loop: the follower is
 * driven by {@code f.update()}, which {@code MyRobot.endPeriodic} calls every loop.
 *
 * <p><b>Why the path is built inside the InstantCommand</b> - the robot's pose is not seeded until
 * {@code MyRobot.onStart()} runs, which happens <em>after</em> {@code atStart()} has built the
 * command group. Build the path when the command runs, not when the sequence is assembled, or you
 * will plan a path from the wrong place. Real autos avoid this by using fixed field poses from
 * {@code AutoPaths} instead of the current pose.
 *
 * <p><b>Where the start pose comes from</b> - {@code onStart()} seeds the follower from
 * {@code MyRobot.autoEndPose}, a static that carries over from whatever ran last, so an auto always
 * sets it with {@code overrideAutoEndPose(...)} first. This test does the same, for repeatability.
 */
@Autonomous(name = "C8 Follow path command", group = "Command Test")
public class FollowPathCommandTest extends CommandTestOpMode {

    /** Bottom-left corner, facing up the field - the same default MyRobot.onStart() would use. */
    private static final Pose START = new Pose(
            MyRobot.chassisLeftOut, MyRobot.chassisBack, Math.toRadians(90));

    private static final double DISTANCE_IN = 12;

    @Override
    protected String subsystemNote() {
        return "follower only. THE ROBOT WILL DRIVE " + (int) DISTANCE_IN
                + " in forward. Clear space ahead of it.";
    }

    @Override
    public void initialize() {
        r = new MyRobot(hardwareMap, telemetry, gamepad1, gamepad2,
                List.of(SubsystemConfig.FOLLOWER),
                OpModeType.AUTO);
    }

    @Override
    public void atStart() {
        // Must happen before MyRobot.onStart(), which is what actually seeds the follower.
        r.overrideAutoEndPose(START);
        super.atStart();
    }

    @Override
    protected Command testSequence() {
        final Pose[] target = new Pose[1];

        return new SequentialCommandGroup(
                measure("start pose", () -> r.f.getPose().toString()),

                note("building and starting the path"),
                new InstantCommand(() -> {
                    Pose here = r.f.getPose();
                    target[0] = new Pose(
                            here.getX() + DISTANCE_IN * Math.cos(here.getHeading()),
                            here.getY() + DISTANCE_IN * Math.sin(here.getHeading()),
                            here.getHeading());

                    r.f.followPath(r.f.pathBuilder()
                            .addPath(new BezierLine(here, target[0]))
                            .setLinearHeadingInterpolation(here.getHeading(), target[0].getHeading())
                            .build());
                }),
                measure("target pose", () -> target[0] == null ? "not built" : target[0].toString()),

                note("waiting for the follower"),
                new WaitUntilCommand(() -> !r.f.isBusy()).withTimeout(5000),

                // The follower reports not-busy slightly before the robot settles.
                new WaitCommand(500),
                measure("end pose", () -> r.f.getPose().toString()),
                measure("error (in)", () -> target[0] == null ? "n/a"
                        : String.format("%.2f", r.f.getPose().distanceFrom(target[0]))),
                measure("still busy?", () -> String.valueOf(r.f.isBusy())),
                note("done")
        );
    }
}
