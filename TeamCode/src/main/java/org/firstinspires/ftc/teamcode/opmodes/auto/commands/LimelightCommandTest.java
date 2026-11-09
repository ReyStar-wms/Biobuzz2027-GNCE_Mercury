package org.firstinspires.ftc.teamcode.opmodes.auto.commands;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.config.core.MyRobot;
import org.firstinspires.ftc.teamcode.config.core.util.VisionMeasurement;
import org.firstinspires.ftc.teamcode.config.core.util.robothelper.OpModeType;
import org.firstinspires.ftc.teamcode.config.core.util.robothelper.SubsystemConfig;
import org.firstinspires.ftc.teamcode.config.subsystems.Limelight;

import java.util.List;

/**
 * Switches the Limelight between its two jobs and logs what each one sees. Nothing moves.
 *
 * <p><b>What to learn here</b> - the camera does two unrelated things and cannot do both at once,
 * because they are different pipelines on the Limelight itself:
 *
 * <ul>
 *   <li>{@code Mode.LOCALIZATION} - AprilTags, which produce a {@link VisionMeasurement} (a field
 *       pose with a timestamp and a quality) for the pose filter to fold in.</li>
 *   <li>{@code Mode.BALL_DETECTION} - the neural detector, which produces a list of artifact
 *       detections that {@code BallLocalizer} turns into tracked balls on the field.</li>
 * </ul>
 *
 * Setting the mode does not talk to the camera directly: {@code Limelight.periodic()} notices the
 * change and only then switches the pipeline, and only if it actually differs from the current one.
 * Switching takes a moment, which is why this test waits a second after each change before reading.
 *
 * <p><b>The FOLLOWER is in the subsystem list on purpose.</b> {@code MyRobot.startPeriodic} feeds
 * the camera from inside its FOLLOWER branch - localization needs the robot's heading, and ball
 * tracking needs the robot's pose to project detections onto the field. With LL alone the subsystem
 * would build and poll, but nothing would ever consume its results.
 */
@Autonomous(name = "C9 Limelight command (modes)", group = "Command Test")
public class LimelightCommandTest extends CommandTestOpMode {

    @Override
    protected String subsystemNote() {
        return "limelight + follower. Nothing moves. Point the camera at a tag, then at artifacts.";
    }

    @Override
    public void initialize() {
        r = new MyRobot(hardwareMap, telemetry, gamepad1, gamepad2,
                List.of(SubsystemConfig.LL, SubsystemConfig.FOLLOWER),
                OpModeType.AUTO);
    }

    @Override
    protected Command testSequence() {
        return new SequentialCommandGroup(
                measure("mode at start", () -> String.valueOf(r.ll.getMode())),

                note("mode -> LOCALIZATION (point the camera at an AprilTag)"),
                new InstantCommand(() -> r.ll.setMode(Limelight.Mode.LOCALIZATION)),
                new WaitCommand(2000),
                measure("tag detected?", () -> String.valueOf(r.ll.isAprilTagDetected())),
                measure("data valid?", () -> String.valueOf(r.ll.isDataValid())),
                measure("measurement", this::describeMeasurement),

                note("mode -> BALL_DETECTION (point the camera at some artifacts)"),
                new InstantCommand(() -> r.ll.setMode(Limelight.Mode.BALL_DETECTION)),
                new WaitCommand(2000),
                measure("detections", () -> r.ll.getDetections() == null
                        ? "null (no frame yet)" : String.valueOf(r.ll.getDetections().size())),
                measure("tracked balls", () -> r.ballLocalizer == null
                        ? "no ball localizer" : String.valueOf(r.ballLocalizer.getBalls().size())),

                note("done - the camera is left in BALL_DETECTION, which is its default")
        );
    }

    private String describeMeasurement() {
        VisionMeasurement measurement = r.ll.getMeasurement();
        if (measurement == null) {
            return "none yet";
        }
        return String.format("%s at t=%.2fs, quality %.2f",
                measurement.pose, measurement.timestampSec, measurement.quality);
    }
}
