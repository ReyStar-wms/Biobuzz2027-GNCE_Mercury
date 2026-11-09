package org.firstinspires.ftc.teamcode.opmodes.auto.base;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * Run this one first. Nothing moves: it just asks the hub for every device the robot expects and
 * reports which ones the configuration actually has, plus battery voltage.
 *
 * <p>A FAIL here means the Driver Station config is wrong (or a cable is out), so there is no point
 * running the per-subsystem checks until it is clean.
 */
@Autonomous(name = "00 Check: all devices", group = "Device Check")
public class AllDevicesCheck extends DeviceCheckOpMode {

    @Override
    protected void runChecks() {
        section("Battery");
        check("voltage", () -> {
            double lowest = Double.MAX_VALUE;
            for (VoltageSensor sensor : hardwareMap.voltageSensor) {
                lowest = Math.min(lowest, sensor.getVoltage());
            }
            require(lowest != Double.MAX_VALUE, "no voltage sensor on any hub");
            require(lowest > 11.0, String.format("%.2f V - charge the battery before testing", lowest));
            return String.format("%.2f V", lowest);
        });

        section("Drivetrain");
        for (String name : Devices.DRIVE_MOTORS) {
            device(DcMotorEx.class, name);
        }

        section("Odometry");
        device(GoBildaPinpointDriver.class, Devices.PINPOINT);

        section("Turret");
        device(ServoImplEx.class, Devices.TURRET_SERVO_1);
        device(ServoImplEx.class, Devices.TURRET_SERVO_2);
        device(AnalogInput.class, Devices.TURRET_ENCODER);

        section("Intake");
        device(DcMotorEx.class, Devices.INTAKE_MOTOR);
        device(DcMotorEx.class, Devices.TRANSFER_MOTOR);
        device(Servo.class, Devices.INTAKE_PITCH_SERVO);

        section("Shooter");
        device(DcMotorEx.class, Devices.FLYWHEEL_1);
        device(DcMotorEx.class, Devices.FLYWHEEL_2);
        device(Servo.class, Devices.HOOD_SERVO);
        device(Servo.class, Devices.DOOR_SERVO);

        section("Lift");
        device(Servo.class, Devices.PTO_SERVO);

        section("Storage");
        for (String name : Devices.BEAM_RECEIVERS) {
            device(DigitalChannel.class, name);
        }
        for (String name : Devices.BEAM_EMITTERS) {
            device(DigitalChannel.class, name);
        }

        section("Vision");
        device(Limelight3A.class, Devices.LIMELIGHT);
    }
}
