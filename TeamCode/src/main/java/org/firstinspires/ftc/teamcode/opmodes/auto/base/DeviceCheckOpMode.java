package org.firstinspires.ftc.teamcode.opmodes.auto.base;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.ArrayList;
import java.util.List;

/**
 * Shared plumbing for the one-subsystem-at-a-time hardware checks in this package.
 *
 * <p>Each check OpMode is meant to be run on its own: select it, press PLAY, and read a pass/fail
 * list on the Driver Station. Anything that will physically move asks for a button press first, so
 * selecting the wrong OpMode cannot surprise anyone.
 *
 * <p>These deliberately go at the hardware directly through {@code hardwareMap} instead of building
 * a {@code MyRobot}. A missing or misconfigured device then shows up as one failed line instead of
 * taking the whole OpMode down in the subsystem constructor, which is the entire point of a check.
 *
 * <p>Results are also written to the robot log (filter logcat on {@code DeviceCheck}), so a failure
 * can be read back after the fact.
 */
public abstract class DeviceCheckOpMode extends LinearOpMode {

    private static final String LOG_TAG = "DeviceCheck";

    /** A single check. Return a short detail string for the pass line; throw to fail. */
    protected interface Probe {
        String run() throws Exception;
    }

    private final List<String> lines = new ArrayList<>();
    private String prompt;
    private int passes;
    private int warnings;
    private int failures;

    /** Run the checks for this subsystem. Called once, after PLAY. */
    protected abstract void runChecks();

    /** Shown on the init screen when this OpMode can move something. */
    protected String safetyNote() {
        return null;
    }

    @Override
    public final void runOpMode() {
        telemetry.addLine(getClass().getSimpleName());
        String note = safetyNote();
        if (note != null) {
            telemetry.addLine("!! " + note);
        }
        telemetry.addLine("Press PLAY to run the checks.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        RobotLog.ii(LOG_TAG, "%s starting", getClass().getSimpleName());
        try {
            runChecks();
        } catch (Throwable t) {
            record("FAIL", "check sequence", describe(t));
            RobotLog.ee(LOG_TAG, t, "%s threw", getClass().getSimpleName());
        }
        prompt = null;
        RobotLog.ii(LOG_TAG, "%s done: %d ok, %d warn, %d fail",
                getClass().getSimpleName(), passes, warnings, failures);

        // Hold the results on screen until the drivers press stop.
        while (opModeIsActive()) {
            render();
            sleep(250);
        }
    }

    // ---------------- check helpers ----------------

    /** Blank line plus a heading, to group the lines that follow. */
    protected void section(String title) {
        lines.add("");
        lines.add("-- " + title);
        render();
    }

    /**
     * Looks a device up without throwing. Reports it as configured or missing, and returns
     * {@code null} when it is not there so the caller can skip the checks that need it.
     */
    protected <T> T device(Class<T> type, String configName) {
        T found;
        try {
            found = hardwareMap.tryGet(type, configName);
        } catch (Throwable t) {
            record("FAIL", configName, describe(t));
            return null;
        }
        if (found == null) {
            record("FAIL", configName, "not in the robot configuration (" + type.getSimpleName() + ")");
        } else {
            record("PASS", configName, "configured");
        }
        return found;
    }

    /** Runs one probe, turning an exception into a failed line instead of a crash. */
    protected boolean check(String label, Probe probe) {
        try {
            String detail = probe.run();
            record("PASS", label, detail == null ? "ok" : detail);
            return true;
        } catch (Throwable t) {
            record("FAIL", label, describe(t));
            return false;
        }
    }

    /** Something that is not obviously broken but needs a human to look at it. */
    protected void warn(String label, String detail) {
        record("WARN", label, detail);
    }

    protected void skipped(String label, String why) {
        lines.add(String.format("  --   %-20s %s", label, why));
        render();
    }

    /** Use inside a probe to fail it with a readable message. */
    protected void require(boolean condition, String message) {
        if (!condition) {
            throw new IllegalStateException(message);
        }
    }

    /**
     * Waits for A to go ahead or B to skip, so nothing moves without someone asking for it.
     *
     * @return true to run the step; false if it was skipped or the OpMode is stopping
     */
    protected boolean confirm(String whatWillHappen) {
        prompt = "A = " + whatWillHappen + "    B = skip";
        render();
        while (opModeIsActive()) {
            if (gamepad1.a || gamepad2.a) {
                prompt = null;
                return true;
            }
            if (gamepad1.b || gamepad2.b) {
                prompt = null;
                skipped(whatWillHappen, "skipped");
                return false;
            }
            sleep(20);
        }
        prompt = null;
        return false;
    }

    /** One-line live status while a long step runs; cleared when the step reports its result. */
    protected void status(String text) {
        prompt = text;
        render();
    }

    /**
     * Shows a live reading for a while, for the checks where a human has to do something (push the
     * robot, wave a hand through a beam) and watch the numbers move.
     */
    protected void observe(String instruction, long millis, Probe sample) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.milliseconds() < millis) {
            String value;
            try {
                value = sample.run();
            } catch (Throwable t) {
                value = describe(t);
            }
            prompt = instruction + " -> " + value;
            render();
            sleep(50);
        }
        prompt = null;
    }

    // ---------------- output ----------------

    private void record(String tag, String label, String detail) {
        if ("PASS".equals(tag)) {
            passes++;
        } else if ("WARN".equals(tag)) {
            warnings++;
            RobotLog.ww(LOG_TAG, "%s | %s: %s", getClass().getSimpleName(), label, detail);
        } else {
            failures++;
            RobotLog.ee(LOG_TAG, "%s | %s: %s", getClass().getSimpleName(), label, detail);
        }
        lines.add(String.format("  %-4s %-20s %s", tag, label, detail));
        prompt = null; // a result supersedes whatever live status was on screen
        render();
    }

    private void render() {
        telemetry.addLine(String.format("%s  [%d ok / %d warn / %d fail]",
                getClass().getSimpleName(), passes, warnings, failures));
        for (String line : lines) {
            telemetry.addLine(line);
        }
        if (prompt != null) {
            telemetry.addLine("");
            telemetry.addLine(">> " + prompt);
        } else if (!lines.isEmpty()) {
            telemetry.addLine("");
            telemetry.addLine(verdict());
        }
        telemetry.update();
    }

    private String verdict() {
        if (failures > 0) {
            return failures + " CHECK(S) FAILED";
        }
        return warnings > 0 ? "PASSED, WITH WARNINGS" : "ALL CHECKS PASSED";
    }

    private static String describe(Throwable t) {
        String message = t.getMessage();
        return t.getClass().getSimpleName() + (message == null ? "" : ": " + message);
    }
}
