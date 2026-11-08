package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.core.MyCommandOpMode;
import org.firstinspires.ftc.teamcode.config.core.MyRobot;
import org.firstinspires.ftc.teamcode.config.core.util.robothelper.SubsystemConfig;

import java.util.List;

@TeleOp(group="Test", name="Intake Test")
public class IntakeTest extends MyCommandOpMode {
    @Override
    public void initialize() {
        r = new MyRobot(hardwareMap, telemetry, gamepad1, gamepad2, List.of(SubsystemConfig.FOLLOWER, SubsystemConfig.INTAKE, SubsystemConfig.DOOR));
    }

    @Override
    public void initialize_loop() {

    }

    @Override
    public void run() {
        r.runIntakeTeleop();
        r.runTransferTeleop();
    }
}
