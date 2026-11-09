package org.firstinspires.ftc.teamcode.opmodes.auto.base;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Checks that the Limelight is reachable over Ethernet, streaming, and that each pipeline the robot
 * uses actually returns results. Nothing moves.
 *
 * <p>The pipeline indices are the ones {@code Limelight.periodic} switches between, mirrored in
 * {@link Devices}. Point the camera at an AprilTag (or the obelisk, or some artifacts) while this
 * runs, and the live line shows what each pipeline sees.
 */
@Autonomous(name = "08 Check: limelight", group = "Device Check")
public class LimelightCheck extends DeviceCheckOpMode {

    @Override
    protected void runChecks() {
        section("Device");
        Limelight3A ll = device(Limelight3A.class, Devices.LIMELIGHT);
        if (ll == null) {
            return;
        }

        check("start streaming", () -> {
            ll.setPollRateHz(50);
            ll.start();
            sleep(500);
            require(ll.isRunning(), "start() did not take - the camera may still be booting");
            return "polling at 50 Hz";
        });

        check("connected", () -> {
            // A Limelight that is powered but not on the network reads as not connected here, which
            // is the usual symptom of a bad Ethernet run or a wrong static IP.
            ElapsedTime timer = new ElapsedTime();
            while (opModeIsActive() && timer.milliseconds() < 3000) {
                if (ll.isConnected()) {
                    return String.format("connected after %.0f ms, last update %d ms ago",
                            timer.milliseconds(), ll.getTimeSinceLastUpdate());
                }
                status(String.format("waiting for the camera... %.1f s", timer.seconds()));
                sleep(100);
            }
            throw new IllegalStateException("not connected after 3 s - check the Ethernet run and power");
        });

        check("status", () -> {
            LLStatus status = ll.getStatus();
            require(status != null, "no status frame");
            return String.format("%s, pipeline %d (%s), %.0f fps, cpu %.0f%%, %.0f C",
                    status.getName(), status.getPipelineIndex(), status.getPipelineType(),
                    status.getFps(), status.getCpu(), status.getTemp());
        });

        section("Pipelines");
        pipeline("localization (AprilTag)", ll, Devices.PIPELINE_LOCALIZATION);
        pipeline("motif", ll, Devices.PIPELINE_MOTIF);
        pipeline("ball detector", ll, Devices.PIPELINE_BALL_DETECTOR);

        section("Live");
        check("back to the ball pipeline", () -> {
            ll.pipelineSwitch(Devices.PIPELINE_BALL_DETECTOR);
            sleep(500);
            return "pipeline " + Devices.PIPELINE_BALL_DETECTOR;
        });
        observe("point the camera at some artifacts", 10000, () -> {
            LLResult result = ll.getLatestResult();
            if (result == null) {
                return "no result yet";
            }
            return String.format("valid %s, %d detections, %.0f ms stale",
                    result.isValid(),
                    result.getDetectorResults() == null ? 0 : result.getDetectorResults().size(),
                    (double) result.getStaleness());
        });
    }

    private void pipeline(String label, Limelight3A ll, int index) {
        check(label, () -> {
            require(ll.pipelineSwitch(index), "pipelineSwitch(" + index + ") was rejected");
            sleep(700);

            LLResult result = null;
            ElapsedTime timer = new ElapsedTime();
            while (opModeIsActive() && timer.milliseconds() < 2000) {
                result = ll.getLatestResult();
                if (result != null && result.getStaleness() < 500) {
                    break;
                }
                sleep(50);
            }
            require(result != null, "no frames came back on pipeline " + index);

            int tags = result.getFiducialResults() == null ? 0 : result.getFiducialResults().size();
            int detections = result.getDetectorResults() == null ? 0 : result.getDetectorResults().size();
            return String.format("pipeline %d, valid %s, %d tags, %d detections, tx %.1f, %.0f ms stale",
                    index, result.isValid(), tags, detections, result.getTx(),
                    (double) result.getStaleness());
        });
    }
}
