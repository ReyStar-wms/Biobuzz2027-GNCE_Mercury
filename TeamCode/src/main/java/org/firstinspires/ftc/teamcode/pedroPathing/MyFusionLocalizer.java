package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.telemetry.JoinedTelemetry;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector;

import java.util.ArrayList;
import java.util.List;
import java.util.NavigableMap;
import java.util.TreeMap;
import java.util.Map;

public class MyFusionLocalizer implements Localizer {
    private final Localizer deadReckoning;
    public JoinedTelemetry t;

    private Pose currentPosition;
    private Pose currentVelocity;
    private Pose lastDeadReckoningPose;

    private Matrix P;
    private final Matrix initialP;
    private final Matrix Q;
    private final Matrix R;

    private long lastUpdateTime = -1;
    private final int bufferSize;

    private final NavigableMap<Long, Pose> poseHistory = new TreeMap<>();
    private final NavigableMap<Long, Pose> deltaHistory = new TreeMap<>();
    private final NavigableMap<Long, Matrix> covarianceHistory = new TreeMap<>();

    public MyFusionLocalizer(
            Localizer deadReckoning,
            Pose initialCovariance,
            Pose processVariance,
            Pose measurementVariance,
            int bufferSize
    ) {
        this.deadReckoning = deadReckoning;
        this.currentPosition = new Pose();
        this.currentVelocity = new Pose();

        this.initialP = Matrix.diag(
                initialCovariance.getX(),
                initialCovariance.getY()
        );
        this.P = initialP.copy();

        this.Q = Matrix.diag(
                processVariance.getX(),
                processVariance.getY()
        );
        this.R = Matrix.diag(
                measurementVariance.getX(),
                measurementVariance.getY()
        );

        this.bufferSize = bufferSize;
    }

    @Override
    public void update() {
        deadReckoning.update();

        long now = System.nanoTime();
        Pose deadPose = deadReckoning.getPose().copy();
        currentVelocity = deadReckoning.getVelocity().copy();

        if (lastDeadReckoningPose == null) {
            lastDeadReckoningPose = deadPose.copy();
            currentPosition = deadPose.copy();

            poseHistory.put(now, currentPosition.copy());
            deltaHistory.put(now, new Pose());
            covarianceHistory.put(now, P.copy());

            lastUpdateTime = now;
            trimHistory();
            return;
        }

        Pose delta = getPoseDelta(lastDeadReckoningPose, deadPose);

        currentPosition = applyDelta(currentPosition, delta);
        P = P.plus(Q);

        poseHistory.put(now, currentPosition.copy());
        deltaHistory.put(now, delta.copy());
        covarianceHistory.put(now, P.copy());

        lastDeadReckoningPose = deadPose.copy();
        lastUpdateTime = now;

        trimHistory();
    }

    public void addMeasurement(Pose measuredPose, long timestamp) {
        addMeasurement(measuredPose, timestamp, null);
    }

    public void addMeasurement(Pose measuredPose, long timestamp, Pose measurementVariance) {
        if (poseHistory.isEmpty()) return;

        if (t != null) {
            t.addData("Timestamp", timestamp);
            t.addData("PoseHistory Last", poseHistory.lastKey());
        }

        if (timestamp < poseHistory.firstKey() || timestamp > poseHistory.lastKey()) return;

        Matrix measurementR = measurementVariance == null
                ? R
                : Matrix.diag(
                measurementVariance.getX(),
                measurementVariance.getY()
        );

        Pose pastPose = interpolatePose(timestamp, poseHistory);
        if (pastPose == null) return;

        boolean measX = !Double.isNaN(measuredPose.getX());
        boolean measY = !Double.isNaN(measuredPose.getY());

        Matrix y = new Matrix(new double[][]{
                {measX ? measuredPose.getX() - pastPose.getX() : 0.0},
                {measY ? measuredPose.getY() - pastPose.getY() : 0.0}
        });

        Matrix M = Matrix.diag(
                measX ? 1.0 : 0.0,
                measY ? 1.0 : 0.0
        );

        Map.Entry<Long, Matrix> covEntry = covarianceHistory.floorEntry(timestamp);
        if (covEntry == null) return;

        Matrix Pm = covEntry.getValue().copy();
        Matrix S = Pm.plus(measurementR);
        Matrix K = M.multiply(Pm.multiply(S.inverse()));
        y = M.multiply(y);

        Matrix Ky = K.multiply(y);
        Pose updatedPast = new Pose(
                pastPose.getX() + Ky.get(0, 0),
                pastPose.getY() + Ky.get(1, 0),
                pastPose.getHeading()
        );

        Matrix I = Matrix.identity(2);
        Matrix IK = I.minus(K);
        Matrix updatedCovariance = IK.multiply(Pm).multiply(IK.transposed())
                .plus(K.multiply(measurementR).multiply(K.transposed()));

        poseHistory.put(timestamp, updatedPast.copy());
        covarianceHistory.put(timestamp, updatedCovariance.copy());

        List<Long> futureTimes = new ArrayList<>(poseHistory.tailMap(timestamp, false).keySet());

        Pose replayPose = updatedPast.copy();
        Matrix replayCovariance = updatedCovariance.copy();

        for (Long tKey : futureTimes) {
            Pose delta = deltaHistory.get(tKey);
            if (delta == null) continue;

            replayPose = applyDelta(replayPose, delta);
            replayCovariance = replayCovariance.plus(Q);

            poseHistory.put(tKey, replayPose.copy());
            covarianceHistory.put(tKey, replayCovariance.copy());
        }

        currentPosition = poseHistory.lastEntry().getValue().copy();
        P = covarianceHistory.lastEntry().getValue().copy();

        if (t != null) {
            t.addLine("Actually Fusioned");
        }
    }

    private void trimHistory() {
        while (poseHistory.size() > bufferSize) poseHistory.pollFirstEntry();
        while (deltaHistory.size() > bufferSize) deltaHistory.pollFirstEntry();
        while (covarianceHistory.size() > bufferSize) covarianceHistory.pollFirstEntry();
    }

    private static Pose getPoseDelta(Pose from, Pose to) {
        return new Pose(
                to.getX() - from.getX(),
                to.getY() - from.getY(),
                0.0
        );
    }

    private static Pose applyDelta(Pose pose, Pose delta) {
        return new Pose(
                pose.getX() + delta.getX(),
                pose.getY() + delta.getY(),
                pose.getHeading()
        );
    }

    private static Pose interpolatePose(long timestamp, NavigableMap<Long, Pose> history) {
        Long lowerKey = history.floorKey(timestamp);
        Long upperKey = history.ceilingKey(timestamp);

        if (lowerKey == null || upperKey == null) return null;
        if (lowerKey.equals(upperKey)) return history.get(lowerKey).copy();

        Pose lowerPose = history.get(lowerKey);
        Pose upperPose = history.get(upperKey);

        double ratio = (double) (timestamp - lowerKey) / (double) (upperKey - lowerKey);

        double x = lowerPose.getX() + ratio * (upperPose.getX() - lowerPose.getX());
        double y = lowerPose.getY() + ratio * (upperPose.getY() - lowerPose.getY());
        double heading = lowerPose.getHeading() + ratio * (upperPose.getHeading() - lowerPose.getHeading());

        return new Pose(x, y, heading);
    }

    private void resetState(Pose pose) {
        currentPosition = pose.copy();
        currentVelocity = new Pose();
        lastDeadReckoningPose = pose.copy();
        P = initialP.copy();
        lastUpdateTime = -1;

        poseHistory.clear();
        deltaHistory.clear();
        covarianceHistory.clear();
    }

    @Override
    public Pose getPose() {
        return new Pose(
                currentPosition.getX(),
                currentPosition.getY(),
                deadReckoning.getPose().getHeading()
        );
    }

    @Override
    public Pose getVelocity() {
        return currentVelocity != null ? currentVelocity : deadReckoning.getVelocity();
    }

    @Override
    public Vector getVelocityVector() {
        return getVelocity().getAsVector();
    }

    @Override
    public void setStartPose(Pose setStart) {
        deadReckoning.setStartPose(setStart);
        resetState(setStart);
    }

    @Override
    public void setPose(Pose setPose) {
        deadReckoning.setPose(setPose);
        resetState(setPose);
    }

    @Override
    public double getTotalHeading() {
        return deadReckoning.getPose().getHeading();
    }

    @Override
    public double getForwardMultiplier() {
        return deadReckoning.getForwardMultiplier();
    }

    @Override
    public double getLateralMultiplier() {
        return deadReckoning.getLateralMultiplier();
    }

    @Override
    public double getTurningMultiplier() {
        return deadReckoning.getTurningMultiplier();
    }

    @Override
    public void resetIMU() throws InterruptedException {
        deadReckoning.resetIMU();
    }

    @Override
    public double getIMUHeading() {
        return deadReckoning.getIMUHeading();
    }

    public Pose getDeadReckoningPose() {
        return deadReckoning.getPose();
    }

    @Override
    public boolean isNAN() {
        return Double.isNaN(currentPosition.getX())
                || Double.isNaN(currentPosition.getY())
                || Double.isNaN(currentPosition.getHeading());
    }
}