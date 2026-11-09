package org.firstinspires.ftc.teamcode.opmodes.auto.commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.config.core.MyCommandOpMode;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

/**
 * Base class for the command-framework walkthroughs in this package. Start here if you are new to
 * this codebase.
 *
 * <h2>How an OpMode on this robot is put together</h2>
 *
 * An OpMode here is deliberately tiny. All the robot's behaviour lives in {@code MyRobot} and in the
 * subsystems; an OpMode only says <em>which</em> parts to build and <em>what</em> to do with them:
 *
 * <ol>
 *   <li>{@code initialize()} runs once when you press INIT. Build a {@code MyRobot} and give it the
 *       list of subsystems this OpMode needs. Only the ones you list get constructed, so a test can
 *       ask for the intake alone and ignore the rest of the robot.</li>
 *   <li>{@code initialize_loop()} then runs repeatedly until PLAY. {@code MyRobot} uses it for
 *       alliance selection and mode toggles; these tests leave it empty.</li>
 *   <li>{@code atStart()} runs once on PLAY. This is where you <em>schedule</em> commands.</li>
 *   <li>{@code run()} runs every loop afterwards, and ticking the command scheduler is what makes
 *       scheduled commands actually advance.</li>
 * </ol>
 *
 * <h2>Why commands instead of writing it inline</h2>
 *
 * A command is a unit of behaviour with a beginning and an end, and the scheduler advances all of
 * them a little bit on every loop. That is what lets the robot do several things at once without any
 * of the code blocking: you describe "spin the flywheel up, and once it is at speed open the door"
 * rather than sitting in a {@code while} loop waiting. The pieces you will use constantly:
 *
 * <ul>
 *   <li>{@code InstantCommand} - do one thing, finish immediately.</li>
 *   <li>{@code WaitCommand} - do nothing for N milliseconds.</li>
 *   <li>{@code WaitUntilCommand} - do nothing until a condition is true. Add
 *       {@code .withTimeout(ms)} so a sensor that never reports cannot hang the whole auto.</li>
 *   <li>{@code SequentialCommandGroup} - run these one after another.</li>
 *   <li>{@code ParallelCommandGroup} - run these at the same time.</li>
 * </ul>
 *
 * Reusable sequences get their own class in {@code config/commands} (see {@code TransferCommand} or
 * {@code OuttakeCommand}), and {@code MyRobot} exposes the big ones as factory methods like
 * {@code r.shootAll()}.
 *
 * <h2>What this base class adds</h2>
 *
 * Each test implements {@link #testSequence()} and returns the commands to run; this class schedules
 * them on PLAY and shows a timestamped log on the Driver Station so you can see which step the robot
 * reached and what the sensors said. Use {@link #note} to mark a step and {@link #measure} to record
 * a reading, both of which are just commands you drop into the sequence.
 *
 * <p>The log also goes to the robot log, so you can read it back afterwards with
 * {@code adb logcat -s CommandTest}.
 *
 * <p>Real autos (see {@code RapidShootTest} or {@code CloseSoloAuto}) call {@code schedule(...)} in
 * {@code atStart()} directly instead of going through a base class like this one. Same mechanism.
 */
public abstract class CommandTestOpMode extends MyCommandOpMode {

    private static final String LOG_TAG = "CommandTest";

    private final List<String> log = new ArrayList<>();
    private final ElapsedTime sinceStart = new ElapsedTime();

    /** The commands this test runs. Called once, on PLAY. */
    protected abstract Command testSequence();

    /** Robot config this test needs; shown on the init screen so you know what to connect. */
    protected String subsystemNote() {
        return null;
    }

    @Override
    public void initialize_loop() {
        // Runs repeatedly between INIT and PLAY. MyRobot.startInitLoop already handles alliance
        // selection and the mode toggles, so all this adds is a reminder of what the test does.
        String note = subsystemNote();
        if (note != null) {
            r.t.addLine("TEST: " + note);
        }
    }

    @Override
    public void atStart() {
        sinceStart.reset();
        log.clear();
        log("started");
        schedule(testSequence());
    }

    @Override
    public void run() {
        // Ticks the command scheduler, which is what advances every scheduled command by one step.
        // (MyRobot.endPeriodic() ticks it a second time each loop - that is how this codebase has
        // always run, and it is harmless, but it is why a command's execute() is called twice per
        // loop if you ever go counting.)
        super.run();

        r.t.addLine();
        r.t.addLine("== " + getClass().getSimpleName() + " ==");
        for (String line : log) {
            r.t.addLine(line);
        }
    }

    // ---------------- logging, as commands ----------------

    /** A command that records a step marker and finishes immediately. */
    protected Command note(String message) {
        return new InstantCommand(() -> log(message));
    }

    /**
     * A command that records a reading. The value is read when the command runs, not when the
     * sequence is built, which is the whole point - build once on PLAY, sample later.
     */
    protected Command measure(String label, Supplier<String> value) {
        return new InstantCommand(() -> {
            String sampled;
            try {
                sampled = value.get();
            } catch (Throwable t) {
                sampled = "FAILED: " + t.getClass().getSimpleName()
                        + (t.getMessage() == null ? "" : " " + t.getMessage());
            }
            log(label + " = " + sampled);
        });
    }

    protected void log(String message) {
        String line = String.format("%6.2fs  %s", sinceStart.seconds(), message);
        log.add(line);
        RobotLog.ii(LOG_TAG, "%s | %s", getClass().getSimpleName(), line);
    }
}
