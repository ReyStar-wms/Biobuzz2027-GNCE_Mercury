package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.core.MyCommandOpMode;
import org.firstinspires.ftc.teamcode.config.core.MyRobot;
import org.firstinspires.ftc.teamcode.config.core.util.robothelper.SubsystemConfig;

import java.util.List;

@Configurable
@TeleOp(name="Bare Shooter Test", group="Test")
public class OuttakeTest extends MyCommandOpMode {

    @Override
    public void initialize() {
        r = new MyRobot(hardwareMap, telemetry, gamepad1, gamepad2, List.of(SubsystemConfig.FOLLOWER, SubsystemConfig.TURRET, SubsystemConfig.SHOOTER));
    }

    @Override
    public void atStart() {
        r.startDrive();
    }

    @Override
    public void run() {
        r.driveControls();
        r.runShootTeleop();
    }
}
