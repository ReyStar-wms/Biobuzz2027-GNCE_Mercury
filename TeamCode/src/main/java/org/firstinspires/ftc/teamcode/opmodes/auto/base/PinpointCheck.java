package org.firstinspires.ftc.teamcode.opmodes.auto.base;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Checks the goBILDA Pinpoint: that it answers on I2C, reports a healthy status, runs at a sane
 * update rate, and that both odometry pods actually count.
 *
 * <p>Nothing moves on its own - after the automatic checks it shows live encoder counts for ten
 * seconds, so push the robot forward and then sideways and watch both numbers change. A pod whose
 * count stays at zero is unplugged or not turning.
 */
@Autonomous(name = "02 Check: pinpoint odometry", group = "Device Check")
public class PinpointCheck extends DeviceCheckOpMode {

    @Override
    protected void runChecks() {
        section("Device");
        GoBildaPinpointDriver odo = device(GoBildaPinpointDriver.class, Devices.PINPOINT);
        if (odo == null) {
            return;
        }

        check("identify", () -> {
            odo.update();
            return String.format("id %d, version %d, yaw scalar %.4f",
                    odo.getDeviceID(), odo.getDeviceVersion(), odo.getYawScalar());
        });

        check("status", () -> {
            odo.update();
            GoBildaPinpointDriver.DeviceStatus status = odo.getDeviceStatus();
            require(status == GoBildaPinpointDriver.DeviceStatus.READY,
                    "status is " + status + " (READY expected)");
            return status.toString();
        });

        check("update rate", () -> {
            for (int i = 0; i < 10; i++) {
                odo.update();
                sleep(20);
            }
            double hz = odo.getFrequency();
            require(hz > 0, "reporting 0 Hz - the Pinpoint is not sampling");
            return String.format("%.0f Hz, loop %d us", hz, odo.getLoopTime());
        });

        section("Pose");
        check("reset and read", () -> {
            odo.resetPosAndIMU();
            sleep(300);
            odo.update();
            return String.format("(%.2f, %.2f) in, %.2f deg",
                    odo.getPosX(DistanceUnit.INCH),
                    odo.getPosY(DistanceUnit.INCH),
                    odo.getHeading(AngleUnit.DEGREES));
        });

        section("Pods");
        final int[] start = new int[2];
        check("baseline encoders", () -> {
            odo.update();
            start[0] = odo.getEncoderX();
            start[1] = odo.getEncoderY();
            return String.format("x %d, y %d", start[0], start[1]);
        });

        observe("push the robot around (forward, then sideways)", 10000, () -> {
            odo.update();
            return String.format("x %+d   y %+d   pose (%.1f, %.1f) %.0f deg",
                    odo.getEncoderX() - start[0],
                    odo.getEncoderY() - start[1],
                    odo.getPosX(DistanceUnit.INCH),
                    odo.getPosY(DistanceUnit.INCH),
                    odo.getHeading(AngleUnit.DEGREES));
        });

        check("both pods counted", () -> {
            odo.update();
            int movedX = Math.abs(odo.getEncoderX() - start[0]);
            int movedY = Math.abs(odo.getEncoderY() - start[1]);
            require(movedX > 100, "the forward pod barely moved (" + movedX + " ticks)");
            require(movedY > 100, "the strafe pod barely moved (" + movedY + " ticks)");
            return String.format("forward %d ticks, strafe %d ticks", movedX, movedY);
        });
    }
}
