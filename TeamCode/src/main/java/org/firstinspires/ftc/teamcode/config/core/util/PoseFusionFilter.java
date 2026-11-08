package org.firstinspires.ftc.teamcode.config.core.util;


import com.pedropathing.geometry.Pose;

public class PoseFusionFilter {
    private final BiasKalman1D xFilter;
    private final BiasKalman1D yFilter;
    private final BiasKalman1D headingFilter;

    private Pose fusedPose;

    public PoseFusionFilter(
            double xVisionStdDev,
            double yVisionStdDev,
            double headingVisionStdDev,
            double xBiasRandomWalkStdDev,
            double yBiasRandomWalkStdDev,
            double headingBiasRandomWalkStdDev,
            double initialBiasStdDev
    ) {
        xFilter = new BiasKalman1D(xVisionStdDev, xBiasRandomWalkStdDev, initialBiasStdDev, false);
        yFilter = new BiasKalman1D(yVisionStdDev, yBiasRandomWalkStdDev, initialBiasStdDev, false);
        headingFilter = new BiasKalman1D(headingVisionStdDev, headingBiasRandomWalkStdDev, initialBiasStdDev, true);
    }

    public void reset(Pose initialPose) {
        xFilter.reset();
        yFilter.reset();
        headingFilter.reset();
        fusedPose = copyPose(initialPose);
    }

    public Pose update(Pose pinpointPose, double dtSeconds) {
        predict(dtSeconds);
        fusedPose = fuse(pinpointPose);
        return copyPose(fusedPose);
    }

    public Pose update(Pose pinpointPose, Pose visionPose, double dtSeconds) {
        predict(dtSeconds);
        correct(pinpointPose, visionPose);
        fusedPose = fuse(pinpointPose);
        return copyPose(fusedPose);
    }

    public Pose getFusedPose() {
        return copyPose(fusedPose);
    }

    public double getXBias() {
        return xFilter.getBias();
    }

    public double getYBias() {
        return yFilter.getBias();
    }

    public double getHeadingBias() {
        return headingFilter.getBias();
    }

    public double getXVariance() {
        return xFilter.getVariance();
    }

    public double getYVariance() {
        return yFilter.getVariance();
    }

    public double getHeadingVariance() {
        return headingFilter.getVariance();
    }

    private void predict(double dtSeconds) {
        xFilter.predict(dtSeconds);
        yFilter.predict(dtSeconds);
        headingFilter.predict(dtSeconds);
    }

    private void correct(Pose pinpointPose, Pose visionPose) {
        xFilter.correct(pinpointPose.getX(), visionPose.getX());
        yFilter.correct(pinpointPose.getY(), visionPose.getY());
        headingFilter.correct(pinpointPose.getHeading(), visionPose.getHeading());
    }

    private Pose fuse(Pose pinpointPose) {
        return new Pose(
                xFilter.corrected(pinpointPose.getX()),
                yFilter.corrected(pinpointPose.getY()),
                headingFilter.corrected(pinpointPose.getHeading())
        );
    }

    private static Pose copyPose(Pose pose) {
        return pose == null ? null : new Pose(pose.getX(), pose.getY(), pose.getHeading());
    }

    private static double wrapAngle(double angle) {
        while (angle <= -Math.PI) angle += 2.0 * Math.PI;
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        return angle;
    }

    private static class BiasKalman1D {
        private final double measurementVariance;
        private final double randomWalkVariancePerSecond;
        private final double initialVariance;
        private final boolean angular;

        private double bias;
        private double variance;

        public BiasKalman1D(
                double measurementStdDev,
                double randomWalkStdDev,
                double initialStdDev,
                boolean angular
        ) {
            measurementVariance = measurementStdDev * measurementStdDev;
            randomWalkVariancePerSecond = randomWalkStdDev * randomWalkStdDev;
            initialVariance = initialStdDev * initialStdDev;
            this.angular = angular;
            reset();
        }

        public void reset() {
            bias = 0.0;
            variance = initialVariance;
        }

        public void predict(double dtSeconds) {
            variance += randomWalkVariancePerSecond * dtSeconds;
        }

        public void correct(double pinpointMeasurement, double visionMeasurement) {
            double measurement = subtract(pinpointMeasurement, visionMeasurement);
            double gain = variance / (variance + measurementVariance);

            bias = add(bias, gain * subtract(measurement, bias));
            variance *= 1.0 - gain;
        }

        public double corrected(double pinpointMeasurement) {
            return subtract(pinpointMeasurement, bias);
        }

        public double getBias() {
            return bias;
        }

        public double getVariance() {
            return variance;
        }

        private double add(double a, double b) {
            return angular ? wrapAngle(a + b) : a + b;
        }

        private double subtract(double a, double b) {
            return angular ? wrapAngle(a - b) : a - b;
        }
    }
}