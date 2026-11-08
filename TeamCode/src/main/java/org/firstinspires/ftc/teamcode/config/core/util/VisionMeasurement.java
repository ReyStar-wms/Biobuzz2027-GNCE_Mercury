package org.firstinspires.ftc.teamcode.config.core.util;


import com.pedropathing.geometry.Pose;

public final class VisionMeasurement {
    public final Pose pose;            // inches, radians
    public final double timestampSec;  // capture time (backdated)
    public final double quality;       // >= 1.0 (bigger = trust less)

    public VisionMeasurement(Pose pose, double timestampSec, double quality) {
        this.pose = pose;
        this.timestampSec = timestampSec;
        this.quality = quality;
    }
}

