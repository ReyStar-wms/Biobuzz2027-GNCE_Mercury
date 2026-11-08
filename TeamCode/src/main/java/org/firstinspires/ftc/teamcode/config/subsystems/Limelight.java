package org.firstinspires.ftc.teamcode.config.subsystems;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.config.core.MyRobot;
import org.firstinspires.ftc.teamcode.config.core.SubsysCore;
import org.firstinspires.ftc.teamcode.config.core.util.robothelper.Motif;
import org.firstinspires.ftc.teamcode.config.core.util.VisionMeasurement;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

public class Limelight extends SubsysCore {
    Limelight3A ll;
    LLResult llResult;

    double tx, ty, ta;
    // Offset front: 198.49661mm
    // Offset Up: 217.62101mm
    // Offset Angle Pitch: 10.806923 deg up from vertical
    double nowSec;
    double robotHeading;
    VisionMeasurement latestMeasurement = null;

    public Limelight(){
        ll = h.get(Limelight3A.class, "Limelight");
        ll.setPollRateHz(50);
        ll.start();
        setMode(Mode.BALL_DETECTION);
    }

    public static double LIMELIGHT_HEIGHT = 16.5; // Inches
    public static double APRILTAG_HEIGHT = 0;
    public static double CAMERA_PITCH = 15; // degrees from vertical

    public void turnOn(){
        ll.start();
    }
    public void turnOff(){
        ll.pause();
    }
    
    public boolean isDataValid(){
        return llResult != null && llResult.isValid();
    }

    public boolean isAprilTagDetected(){ return llResult !=null && llResult.isValid(); }

    public Mode getMode() {
        return mode;
    }

    public enum Mode {
        LOCALIZATION,
        MOTIF_DETECTION,
        BALL_DETECTION
    }

    Mode mode;

    // Limelight pipeline index running the neural ball detector.
    public static int BALL_PIPELINE_INDEX = 3;
    // Latest neural-detector results from the ball pipeline.
    List<LLResultTypes.DetectorResult> latestDetections = null;

    public List<LLResultTypes.DetectorResult> getDetections() {
        return latestDetections;
    }

    public void setMode(Mode mode) {
        this.mode = mode;
    }

    // pipelineSwitch sends a command every call; only send it when the pipeline actually changes.
    private int currentPipeline = -1;
    private void switchPipeline(int idx){
        if(idx != currentPipeline){
            ll.pipelineSwitch(idx);
            currentPipeline = idx;
        }
    }

    public void update(double nowSec, double robotHeading){
        this.nowSec = nowSec;
        this.robotHeading = robotHeading;
    }

    public VisionMeasurement getMeasurement(){
        return latestMeasurement;
    }

    @Override
    public void periodic() {
        // ll.getStatus()/isConnected()/isRunning() are telemetry-only reads; skip them in match mode.
        if(telemetryEnabled){
            LLStatus status = ll.getStatus();
            t.addData("LL Name", "%s", status.getName());
            t.addData("LL State", "Temp: %.1fC, CPU: %.1f%%, FPS: %d", status.getTemp(), status.getCpu(),(int)status.getFps());
            t.addData("Pipeline", "Index: %d, Type: %s", status.getPipelineIndex(), status.getPipelineType());
            t.addData("LL Mode", mode.name());
        }

        switch (mode){
            case LOCALIZATION:
                switchPipeline(1);
                ll.updateRobotOrientation((robotHeading + 90)%360);
                break;
            case MOTIF_DETECTION:
                switchPipeline(0);
                break;
            case BALL_DETECTION:
                switchPipeline(BALL_PIPELINE_INDEX);
                break;
        }

        llResult = ll.getLatestResult();

        if (llResult != null) {
            double captureLatency = llResult.getCaptureLatency();
            double targetingLatency = llResult.getTargetingLatency();

            if(telemetryEnabled){
                t.addData("LL Latency", captureLatency + targetingLatency);
                t.addData("LL Parse Latency", llResult.getParseLatency());
                t.addData("Target X", llResult.getTx());
                t.addData("Target Y", llResult.getTy());
                t.addData("Target Area", llResult.getTa());
            }

            latestMeasurement = null;

            switch (mode){
                case LOCALIZATION:
                    if (llResult.isValid()) {
                        double in = 39.37007874;
                        Pose3D mt2Pose = llResult.getBotpose_MT2();

                        if (mt2Pose != null) {
                            Pose pedro2 = FTCCoordinates.INSTANCE.convertToPedro(new Pose(
                                    mt2Pose.getPosition().x * in,
                                    mt2Pose.getPosition().y * in,
                                    mt2Pose.getOrientation().getYaw(AngleUnit.RADIANS)
                            ));
                            pedro2 = new Pose(pedro2.getX(), pedro2.getY()+1.5, Double.NaN);

                            if(telemetryEnabled){
                                t.addData("MT2 BotPose", mt2Pose.toString());
                                t.addData("MT2 PedroConv", pedro2);
                            }

                            long captureTime = System.nanoTime() - (long)((captureLatency + targetingLatency)*(1e6));
                            Constants.fusionLocalizer.addMeasurement(pedro2, captureTime);
                        }
                    }
                    break;
                case MOTIF_DETECTION:
                    if(MyRobot.currentMotif == null){
                        List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();
                        if(fiducials.size() == 1){
                            int id = fiducials.get(0).getFiducialId();
                            MyRobot.currentMotif = Motif.getMotif(id);
                        }
                    }
                    break;
                case BALL_DETECTION:
                    latestDetections = llResult.getDetectorResults();
                    if(telemetryEnabled) t.addData("Balls Detected", latestDetections != null ? latestDetections.size() : 0);
                    break;
            }
        } else {
            if(telemetryEnabled) t.addLine("AprilTag Not Detected");
        }

        if(telemetryEnabled){
            t.addData("Limelight Connected?", ll.isConnected());
            t.addData("Limelight Running?", ll.isRunning());
        }
    }
}
