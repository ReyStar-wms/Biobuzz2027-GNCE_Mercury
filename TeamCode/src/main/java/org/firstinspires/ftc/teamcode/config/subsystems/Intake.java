package org.firstinspires.ftc.teamcode.config.subsystems;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.config.core.SubsysCore;
import org.firstinspires.ftc.teamcode.config.hardware.CachedMotor;

@Configurable
public class Intake extends SubsysCore {
    CachedMotor im, tr;
    ServoEx servo;
    double pwr;

    public static double INTAKE_DOWN = 0.68; // 0.47
    public static double INTAKE_UP = 0.68;
    public static double TRANSFER_I_POWER = 1;
    public static double TRANSFER_T_POWER = 1;
    public static double ONE_TRANSFER_T_POWER = 1;

    public static double TRANSFER_POWER_WHEN_INTAKE = 0.45;

    Timer timer;

    public Intake(){
        im = new CachedMotor(h.get(DcMotorEx.class, "intakeMotor"));
        im.setDirection(DcMotorSimple.Direction.REVERSE);

        tr = new CachedMotor(h.get(DcMotorEx.class, "transferMotor"));
        tr.setDirection(DcMotorSimple.Direction.REVERSE);

        servo = new ServoEx(h, "intakePitch");
        pwr = 0;
        mode = Mode.INTAKE;
        setDefaultCommand(new RunCommand(() -> mode = Mode.INTAKE, this));
        timer = new Timer();
        timer.resetTimer();
    }

    private double lastPitch = Double.NaN;
    public void setIntakePitch(double sp){
        // Cache the write: the pitch servo holds position, so skip re-sending the same value each loop.
        if(Double.isNaN(lastPitch) || sp != lastPitch){
            servo.set(sp);
            lastPitch = sp;
        }
    }

    public double getCurrent(){
        return im.getCurrent();
    }

    public enum Mode {
        INTAKE,
        TRANSFER,
        DISABLE,
        REVERSE
    }

    Mode mode;

    int size;
    int prevsize;

    public void inputStorageSize(int size){
        this.size = size;
    }
    public void setMode(Mode mode){
        this.mode = mode;
    }
    public double getIntakeVelocity(){
        return im.getVelocity();
    }

    @Override
    public void periodic() {
        switch (mode) {
            case DISABLE:
                im.setPower(0);
                tr.setPower(0);
                break;
            case REVERSE:
                im.setPower(-1);
                tr.setPower(-1);
                break;
            case INTAKE:
                if (size <= 1) tr.setPower(TRANSFER_POWER_WHEN_INTAKE);
                else tr.setPower(0);
                if (size >= 3){
                    im.setPower(0);
                    setIntakePitch(INTAKE_UP);
                } else {
                    im.setPower(1);
                    setIntakePitch(INTAKE_DOWN);
                }
                break;
            case TRANSFER:
                im.setPower(TRANSFER_I_POWER);
                if(size <= 1) tr.setPower(ONE_TRANSFER_T_POWER);
                else tr.setPower(TRANSFER_T_POWER);
                break;
        }
        if(mode != Mode.INTAKE && mode != Mode.DISABLE) setIntakePitch(INTAKE_UP);
        // getCurrent() is a non-bulk hub round-trip; only pay for it when telemetry is being shown.
        if(telemetryEnabled){
            t.addData("Intake Mode", mode.name());
            t.addData("Intake Current", im.getCurrent());
            t.addData("Intake Velocity", im.getVelocity());
            t.addData("Transfer Current", tr.getCurrent());
        }
        prevsize = size;
    }
}
