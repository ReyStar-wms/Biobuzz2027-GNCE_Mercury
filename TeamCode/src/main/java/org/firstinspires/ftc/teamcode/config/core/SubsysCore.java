package org.firstinspires.ftc.teamcode.config.core;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.bylazar.telemetry.JoinedTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SubsysCore extends SubsystemBase {
    public static HardwareMap h;
    public static JoinedTelemetry t;

    // Mirror of MyRobot.telemetryEnabled, pushed each loop. Subsystems gate their telemetry -- and
    // especially the hardware reads that ONLY feed telemetry (motor current, LL status) -- on this, so
    // those per-loop hub round-trips disappear in match mode. Defaults true so nothing is hidden until
    // the toggle is wired up.
    public static boolean telemetryEnabled = true;

    public SubsysCore(){
        super();
    }

    public static void setGlobalParameters(HardwareMap newH, JoinedTelemetry newT) {
        h = newH;
        t = newT;
    }
}
