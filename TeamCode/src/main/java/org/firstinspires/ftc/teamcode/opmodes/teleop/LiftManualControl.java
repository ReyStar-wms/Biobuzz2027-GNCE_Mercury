package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.config.core.MyCommandOpMode;
import org.firstinspires.ftc.teamcode.config.core.MyRobot;
import org.firstinspires.ftc.teamcode.config.core.util.robothelper.OpModeType;
import org.firstinspires.ftc.teamcode.config.core.util.robothelper.SubsystemConfig;
import org.firstinspires.ftc.teamcode.config.subsystems.Lift;

import java.util.List;

@TeleOp(name="Lift Manual", group="Utils")
public class LiftManualControl extends MyCommandOpMode {
    @Override
    public void initialize() {
        r = new MyRobot(hardwareMap, telemetry, gamepad1, gamepad2, List.of(SubsystemConfig.LIFT), OpModeType.TELEOP);
    }

    @Override
    public void run() {
        if(r.g1.wasJustPressed(GamepadKeys.Button.TRIANGLE)) r.lift.setMode(Lift.Mode.RAW);
        if(r.g1.wasJustPressed(GamepadKeys.Button.SQUARE)) r.lift.setMode(Lift.Mode.INACTIVE);
        if(r.g1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) schedule(r.lift.FullLiftCommand());
        r.lift.setManualPower(r.g1.getLeftY());
    }
}
