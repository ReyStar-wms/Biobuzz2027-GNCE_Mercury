package org.firstinspires.ftc.teamcode.config.core.util;

import java.util.Arrays;

import com.pedropathing.geometry.Pose;

public final class PoseEKF {

    // Tunables
    public double baseVisionPosStd = 2.0;                   // inches
    public double baseVisionHeadingStd = Math.toRadians(1.5);// rad. setting to 45 does nothing.
    public double visionPosStdPerInPerSec = 0.08;
    public double visionHeadingStdPerRadPerSec = 0.60;

    public double baseOdoPosStd = 0.05;                     // inches per step
    public double baseOdoHeadingStd = Math.toRadians(0.10); // rad per step
    public double odoPosStdPerInPerSec = 0.002;
    public double odoHeadingStdPerRadPerSec = 0.01;

    public double outlierDistanceIn = 12.0;
    public double outlierHeadingRad = Math.toRadians(25.0);
    public double outlierInflation = 25.0;

    public int bufferCapacity = 250;
    public double bufferMaxAgeSec = 1.2;

    // State
    private final Vec3 x = new Vec3(0, 0, 0);
    private final Mat3 P = Mat3.identity().scale(1e-3);

    // For absolute-odo delta computation (Pinpoint via f.getPose())
    private boolean haveLastOdo = false;
    private double lastOdoT = 0;
    private double lastOdoX = 0, lastOdoY = 0, lastOdoH = 0;

    // Ring buffer
    private Sample[] ring;
    private int ringHead = 0;
    private int ringSize = 0;

    public PoseEKF() {
        allocRing();
    }

    private void allocRing() {
        ring = new Sample[Math.max(64, bufferCapacity)];
        for (int i = 0; i < ring.length; i++) ring[i] = new Sample();
        ringHead = 0;
        ringSize = 0;
    }

    public void resetPose(Pose pose) {
        x.x = pose.getX();
        x.y = pose.getY();
        x.z = wrap(pose.getHeading());

        P.set(Mat3.identity().scale(1e-3));

        haveLastOdo = false;
        clearBuffer();
    }

    public Pose getPose() {
        return new Pose(x.x, x.y, x.z);
    }

    public void clearBuffer() {
        ringHead = 0;
        ringSize = 0;
        for (Sample s : ring) s.reset();
    }

    /**
     * Predict using ABSOLUTE odometry pose (from Pedro follower f.getPose()).
     * If speed/omega are NaN, EKF estimates them from deltas.
     */
    public void predictFromAbsoluteOdo(double nowSec, Pose odoPose,
                                       double speedInPerSec, double omegaRadPerSec) {
        double odoX = odoPose.getX();
        double odoY = odoPose.getY();
        double odoH = wrap(odoPose.getHeading());

        if (!haveLastOdo) {
            haveLastOdo = true;
            lastOdoT = nowSec;
            lastOdoX = odoX;
            lastOdoY = odoY;
            lastOdoH = odoH;

            resetPose(new Pose(odoX, odoY, odoH));
            return;
        }

        double dt = nowSec - lastOdoT;
        if (dt <= 1e-6) return;

        double dx = odoX - lastOdoX;
        double dy = odoY - lastOdoY;
        double dH = angleDiff(odoH, lastOdoH);

        double estSpeed = Math.hypot(dx, dy) / dt;
        double estOmega = Math.abs(dH) / dt;

        if (!Double.isFinite(speedInPerSec)) speedInPerSec = estSpeed;
        if (!Double.isFinite(omegaRadPerSec)) omegaRadPerSec = estOmega;

        predictDelta(nowSec, dx, dy, dH, speedInPerSec, omegaRadPerSec);

        lastOdoT = nowSec;
        lastOdoX = odoX;
        lastOdoY = odoY;
        lastOdoH = odoH;
    }

    public void predictDelta(double nowSec,
                             double dxFieldIn, double dyFieldIn, double dThetaRad,
                             double speedInPerSec, double omegaRadPerSec) {
        x.x += dxFieldIn;
        x.y += dyFieldIn;
        x.z = wrap(x.z + dThetaRad);

        P.addInPlace(processNoiseQ(speedInPerSec, omegaRadPerSec));

        pushSample(nowSec, dxFieldIn, dyFieldIn, dThetaRad, speedInPerSec, omegaRadPerSec, x, P);
    }

    /**
     * Latency-compensated vision update.
     * Pass visionPose as a Pedro Pose (inches, radians).
     * visionTimestampSec should be now - (captureLatency+targetingLatency)/1000.
     */
    public void updateVisionWithLatency(double nowSec,
                                        Pose visionPose,
                                        double visionTimestampSec,
                                        double speedInPerSec, double omegaRadPerSec,
                                        double qualityScale) {

        double vX = visionPose.getX();
        double vY = visionPose.getY();
        double vH = wrap(visionPose.getHeading());

        if (ringSize == 0) {
            ekfUpdate(vX, vY, vH, speedInPerSec, omegaRadPerSec, qualityScale);
            return;
        }

        double tMeas = clamp(visionTimestampSec, ringOldestTime(), ringNewestTime());
        int idx = findNearestSampleIndex(tMeas);
        Sample base = ring[idx];

        // Rewind
        x.set(base.x);
        P.set(base.P);

        if (!Double.isFinite(speedInPerSec)) speedInPerSec = base.speed;
        if (!Double.isFinite(omegaRadPerSec)) omegaRadPerSec = base.omega;

        Mat3 R = measurementNoiseR(speedInPerSec, omegaRadPerSec, qualityScale);

        // Soft outlier inflate
        double dist = Math.hypot(vX - x.x, vY - x.y);
        double dh = Math.abs(angleDiff(vH, x.z));
        if (dist > outlierDistanceIn || dh > outlierHeadingRad) {
            R = new Mat3(R).scale(outlierInflation);
        }

        // Update
        ekfUpdate(vX, vY, vH, R);

        // Replay to newest
        int newestIdx = ringNewestIndex();
        int cur = idx;
        while (cur != newestIdx) {
            cur = ringNextIndex(cur);
            Sample s = ring[cur];

            x.x += s.dx;
            x.y += s.dy;
            x.z = wrap(x.z + s.dH);

            P.addInPlace(processNoiseQ(s.speed, s.omega));
        }

        overwriteNewestWithCurrent();
    }

    // ---------------- EKF math ----------------

    private void ekfUpdate(double measX, double measY, double measH,
                           double speed, double omega, double qualityScale) {
        if (!Double.isFinite(speed)) speed = 0.0;
        if (!Double.isFinite(omega)) omega = 0.0;
        ekfUpdate(measX, measY, measH, measurementNoiseR(speed, omega, qualityScale));
    }

    private void ekfUpdate(double measX, double measY, double measH, Mat3 R) {
        Vec3 y = new Vec3(
                measX - x.x,
                measY - x.y,
                angleDiff(wrap(measH), x.z)
        );

        Mat3 S = new Mat3(P).addInPlace(new Mat3(R));
        Mat3 K = new Mat3(P).mul(S.inverseSafe());

        Vec3 Ky = K.mul(y);
        x.x += Ky.x;
        x.y += Ky.y;
        x.z = wrap(x.z + Ky.z);

        Mat3 newP = Mat3.identity().sub(K).mul(P);
        newP.symmetrizeInPlace();
        P.set(newP);
    }

    private Mat3 processNoiseQ(double speed, double omega) {
        double posStd = baseOdoPosStd + odoPosStdPerInPerSec * Math.abs(speed);
        double headStd = baseOdoHeadingStd + odoHeadingStdPerRadPerSec * Math.abs(omega);
        return Mat3.diag(posStd * posStd, posStd * posStd, headStd * headStd);
    }

    private Mat3 measurementNoiseR(double speed, double omega, double qualityScale) {
        double posStd = baseVisionPosStd + visionPosStdPerInPerSec * Math.abs(speed);
        double headStd = baseVisionHeadingStd + visionHeadingStdPerRadPerSec * Math.abs(omega);

        double q = Math.max(1.0, qualityScale);
        posStd *= q;
        headStd *= q;

        return Mat3.diag(posStd * posStd, posStd * posStd, headStd * headStd);
    }

    // ---------------- Buffer ----------------

    private void pushSample(double t, double dx, double dy, double dH,
                            double speed, double omega, Vec3 xNow, Mat3 PNow) {
        Sample s = ring[ringHead];
        s.t = t;
        s.dx = dx;
        s.dy = dy;
        s.dH = dH;
        s.speed = speed;
        s.omega = omega;
        s.x.set(xNow);
        s.P.set(PNow);

        ringHead = ringNextIndex(ringHead);
        if (ringSize < ring.length) ringSize++;

        while (ringSize > 2 && (ringNewestTime() - ringOldestTime()) > bufferMaxAgeSec) {
            ringSize--;
        }
    }

    private int ringOldestIndex() {
        int idx = ringHead - ringSize;
        if (idx < 0) idx += ring.length;
        return idx;
    }

    private int ringNewestIndex() {
        int idx = ringHead - 1;
        return (idx < 0) ? ring.length - 1 : idx;
    }

    private double ringOldestTime() { return ring[ringOldestIndex()].t; }
    private double ringNewestTime() { return ring[ringNewestIndex()].t; }

    private int ringNextIndex(int i) {
        int n = i + 1;
        return (n >= ring.length) ? 0 : n;
    }

    private int findNearestSampleIndex(double t) {
        int idx = ringOldestIndex();
        int best = idx;
        double bestDt = Math.abs(ring[idx].t - t);

        for (int k = 1; k < ringSize; k++) {
            idx = ringNextIndex(idx);
            double dt = Math.abs(ring[idx].t - t);
            if (dt < bestDt) { bestDt = dt; best = idx; }
        }
        return best;
    }

    private void overwriteNewestWithCurrent() {
        int newest = ringNewestIndex();
        ring[newest].x.set(x);
        ring[newest].P.set(P);
    }

    // ---------------- Helpers ----------------

    public static double wrap(double a) {
        while (a <= -Math.PI) a += 2.0 * Math.PI;
        while (a > Math.PI) a -= 2.0 * Math.PI;
        return a;
    }

    public static double angleDiff(double a, double b) { return wrap(a - b); }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    // ---------------- Small math types ----------------

    private static final class Vec3 {
        double x, y, z;
        Vec3(double x, double y, double z) { this.x = x; this.y = y; this.z = z; }
        void set(Vec3 o) { this.x = o.x; this.y = o.y; this.z = o.z; }
    }

    public static final class Mat3 {
        double[] m = new double[9];

        Mat3() {}
        Mat3(Mat3 o) { set(o); }

        static Mat3 identity() {
            Mat3 r = new Mat3();
            r.m[0]=1; r.m[4]=1; r.m[8]=1;
            return r;
        }

        static Mat3 diag(double a, double b, double c) {
            Mat3 r = new Mat3();
            r.m[0]=a; r.m[4]=b; r.m[8]=c;
            return r;
        }

        Mat3 set(Mat3 o) { System.arraycopy(o.m, 0, m, 0, 9); return this; }

        Mat3 scale(double s) { for (int i=0;i<9;i++) m[i]*=s; return this; }

        Mat3 addInPlace(Mat3 o) { for (int i=0;i<9;i++) m[i]+=o.m[i]; return this; }

        Mat3 sub(Mat3 o) {
            Mat3 r = new Mat3(this);
            for (int i=0;i<9;i++) r.m[i]-=o.m[i];
            return r;
        }

        Mat3 mul(Mat3 o) {
            Mat3 r = new Mat3();
            double[] a = this.m, b = o.m, c = r.m;

            c[0] = a[0]*b[0] + a[1]*b[3] + a[2]*b[6];
            c[1] = a[0]*b[1] + a[1]*b[4] + a[2]*b[7];
            c[2] = a[0]*b[2] + a[1]*b[5] + a[2]*b[8];

            c[3] = a[3]*b[0] + a[4]*b[3] + a[5]*b[6];
            c[4] = a[3]*b[1] + a[4]*b[4] + a[5]*b[7];
            c[5] = a[3]*b[2] + a[4]*b[5] + a[5]*b[8];

            c[6] = a[6]*b[0] + a[7]*b[3] + a[8]*b[6];
            c[7] = a[6]*b[1] + a[7]*b[4] + a[8]*b[7];
            c[8] = a[6]*b[2] + a[7]*b[5] + a[8]*b[8];

            return r;
        }

        Vec3 mul(Vec3 v) {
            return new Vec3(
                    m[0]*v.x + m[1]*v.y + m[2]*v.z,
                    m[3]*v.x + m[4]*v.y + m[5]*v.z,
                    m[6]*v.x + m[7]*v.y + m[8]*v.z
            );
        }

        Mat3 inverseSafe() {
            Mat3 inv = inverse();
            if (!inv.isFinite()) {
                Mat3 reg = new Mat3(this);
                reg.m[0] += 1e-9; reg.m[4] += 1e-9; reg.m[8] += 1e-9;
                inv = reg.inverse();
            }
            return inv;
        }

        private boolean isFinite() {
            for (double v : m) if (!Double.isFinite(v)) return false;
            return true;
        }

        Mat3 inverse() {
            double a00=m[0], a01=m[1], a02=m[2];
            double a10=m[3], a11=m[4], a12=m[5];
            double a20=m[6], a21=m[7], a22=m[8];

            double c00 =  (a11*a22 - a12*a21);
            double c01 = -(a10*a22 - a12*a20);
            double c02 =  (a10*a21 - a11*a20);

            double c10 = -(a01*a22 - a02*a21);
            double c11 =  (a00*a22 - a02*a20);
            double c12 = -(a00*a21 - a01*a20);

            double c20 =  (a01*a12 - a02*a11);
            double c21 = -(a00*a12 - a02*a10);
            double c22 =  (a00*a11 - a01*a10);

            double det = a00*c00 + a01*c01 + a02*c02;

            Mat3 r = new Mat3();
            if (Math.abs(det) < 1e-12) {
                Arrays.fill(r.m, Double.NaN);
                return r;
            }

            double invDet = 1.0 / det;

            r.m[0] = c00*invDet; r.m[1] = c10*invDet; r.m[2] = c20*invDet;
            r.m[3] = c01*invDet; r.m[4] = c11*invDet; r.m[5] = c21*invDet;
            r.m[6] = c02*invDet; r.m[7] = c12*invDet; r.m[8] = c22*invDet;

            return r;
        }

        void symmetrizeInPlace() {
            double m01 = 0.5*(m[1] + m[3]);
            double m02 = 0.5*(m[2] + m[6]);
            double m12 = 0.5*(m[5] + m[7]);
            m[1]=m01; m[3]=m01;
            m[2]=m02; m[6]=m02;
            m[5]=m12; m[7]=m12;
        }
    }

    private static final class Sample {
        double t = Double.NaN;
        double dx = 0, dy = 0, dH = 0;
        double speed = 0, omega = 0;
        final Vec3 x = new Vec3(0,0,0);
        final Mat3 P = Mat3.identity().scale(1e-3);

        void reset() {
            t = Double.NaN;
            dx = dy = dH = 0;
            speed = omega = 0;
            x.x = x.y = x.z = 0;
            P.set(Mat3.identity().scale(1e-3));
        }
    }
}
