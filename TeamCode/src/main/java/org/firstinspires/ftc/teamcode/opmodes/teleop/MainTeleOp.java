package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.core.MyCommandOpMode;
import org.firstinspires.ftc.teamcode.config.core.MyRobot;
import org.firstinspires.ftc.teamcode.config.core.util.robothelper.SubsystemConfig;

import java.util.List;

@Configurable
@TeleOp(group="TeleOp", name="Main TeleOp")
public class MainTeleOp extends MyCommandOpMode {

    @Override
    public void initialize() {
        r = new MyRobot(hardwareMap, telemetry, gamepad1, gamepad2, List.of(SubsystemConfig.INTAKE, SubsystemConfig.TURRET, SubsystemConfig.SHOOTER, SubsystemConfig.DOOR, SubsystemConfig.FOLLOWER, SubsystemConfig.LIFT));
    }

    @Override
    public void initialize_loop() {
    }

    @Override
    public void run() {
        r.driveControls();
        r.runIntakeTeleop();
        r.runShootTeleop();
    }
}
