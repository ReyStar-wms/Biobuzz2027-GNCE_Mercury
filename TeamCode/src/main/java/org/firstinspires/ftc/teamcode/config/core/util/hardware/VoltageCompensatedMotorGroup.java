package org.firstinspires.ftc.teamcode.config.core.util.hardware;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;

public class VoltageCompensatedMotorGroup extends MotorGroup {

    private final HardwareMap hardwareMap;
    private final long voltageCacheTimeMillis;
    private final double nominalVoltage;

    private double cachedVoltage = 12.0;
    private long lastVoltageReadTimeMillis = 0L;

    private double commandedPower = 0.0;

    public VoltageCompensatedMotorGroup(
            @NonNull HardwareMap hardwareMap,
            @NonNull Motor leader,
            Motor... followers
    ) {
        this(hardwareMap, 500L, 12.0, leader, followers);
    }

    public VoltageCompensatedMotorGroup(
            @NonNull HardwareMap hardwareMap,
            long voltageCacheTimeMillis,
            double nominalVoltage,
            @NonNull Motor leader,
            Motor... followers
    ) {
        super(leader, followers);
        this.hardwareMap = hardwareMap;
        this.voltageCacheTimeMillis = voltageCacheTimeMillis;
        this.nominalVoltage = nominalVoltage;
    }

    private double getBatteryVoltage() {
        long now = System.currentTimeMillis();

        if (now - lastVoltageReadTimeMillis >= voltageCacheTimeMillis) {
            double minPositive = Double.POSITIVE_INFINITY;

            for (VoltageSensor sensor : hardwareMap.voltageSensor) {
                double v = sensor.getVoltage();
                if (v > 0 && v < minPositive) {
                    minPositive = v;
                }
            }

            if (minPositive != Double.POSITIVE_INFINITY) {
                cachedVoltage = minPositive;
                lastVoltageReadTimeMillis = now;
            }
        }

        return cachedVoltage;
    }

    @Override
    public void set(double speed) {
        commandedPower = speed;

        double voltage = getBatteryVoltage();
        double compensated = speed * nominalVoltage / voltage;

        compensated = Math.max(-1.0, Math.min(1.0, compensated));

        super.set(compensated);
    }

    @Override
    public double get() {
        return commandedPower;
    }

    public double getAppliedPower() {
        return super.get();
    }

    public double getCachedVoltage() {
        return getBatteryVoltage();
    }
}
