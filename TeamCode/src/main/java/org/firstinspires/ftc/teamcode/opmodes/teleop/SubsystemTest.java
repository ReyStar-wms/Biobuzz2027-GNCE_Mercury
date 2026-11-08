package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.core.MyCommandOpMode;
import org.firstinspires.ftc.teamcode.config.core.MyRobot;
import org.firstinspires.ftc.teamcode.config.core.util.robothelper.SubsystemConfig;

import java.util.List;

@TeleOp(name="Subsystem Test", group = "Test")
public class SubsystemTest extends MyCommandOpMode {
    @Override
    public void initialize() {
        r = new MyRobot(hardwareMap, telemetry, gamepad1, gamepad2, List.of(SubsystemConfig.FOLLOWER, SubsystemConfig.TURRET));
    }

    @Override
    public void atStart() {
        r.startDrive();
    }

    @Override
    public void run() {
        r.driveControls();
    }
}
