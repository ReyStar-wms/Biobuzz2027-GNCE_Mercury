package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.core.MyCommandOpMode;
import org.firstinspires.ftc.teamcode.config.core.MyRobot;
import org.firstinspires.ftc.teamcode.config.core.util.robothelper.SubsystemConfig;

import java.util.List;

/**
 * Drive the robot around and watch every detected ball's field position, velocity, and
 * confidence in telemetry (and plotted on the Panels field). Uses only the FOLLOWER and LL
 * subsystems, with the Limelight in neural ball-detection mode.
 */
@Configurable
@TeleOp(name="Ball Vision TeleOp", group="Test")
public class BallVisionTeleOp extends MyCommandOpMode {

    @Override
    public void initialize() {
        r = new MyRobot(hardwareMap, telemetry, gamepad1, gamepad2,
                List.of(SubsystemConfig.FOLLOWER, SubsystemConfig.LL));
    }

    @Override
    public void atStart() {
        r.startDrive();
        r.enableBallDetection();
    }

    @Override
    public void run() {
        r.driveControls();
        r.ballTelemetry();
    }
}
