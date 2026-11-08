package org.firstinspires.ftc.teamcode.config.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.teamcode.config.core.SubsysCore;

@Configurable
public class Lift extends SubsysCore {
    MotorGroup front, back;
    ServoEx pto;
    private double lastPto = Double.NaN;
    public static int tar = 0;
    public static double kp = 0.9, ki = 0, kd = 0, kf = 0.3;
    public static int positionTolerance = 30;
    public static double CLUTCH_PULLED = 1;
    public static double CLUTCH_MESHED = 0;
    public static double ENGAGING_POWER = 0.5;
    public static long ENGAGING_DELAY = 300;
    PIDFController pidf;

    public enum LiftPositions {
        RETRACTED(0), EXTENDED(3500);

        private final int ticks;
        LiftPositions(int ticks){
            this.ticks = ticks;
        }
    }

    public Lift(){
        front = new MotorGroup(
                new Motor(h, "fl").setInverted(true),
                new Motor(h, "fr")
        );
        back = new MotorGroup(
                new MotorEx(h, "br"),
                new MotorEx(h, "bl").setInverted(true)
        );
        pto = new ServoEx(h, "pto");
        back.setRunMode(Motor.RunMode.RawPower);
        setMode(Mode.INACTIVE);
        setTargetPosition(LiftPositions.RETRACTED);
        pidf = new PIDFController(kp, ki, kd, kf);
        pidf.setTolerance(positionTolerance);
    }

    public void setTargetPosition(LiftPositions liftPositions){
        tar = liftPositions.ticks;
    }

    public enum Mode {
        RAW,
        PID,
        INACTIVE
    }

    public static Mode mode;

    public void setMode(Mode mode) {
        Lift.mode = mode;
    }

    double pwr = 0;

    public void setManualPower(double pwr){
        this.pwr = pwr;
    }
    public int getCurrentPosition(){
        return back.getMotor().getCurrentPosition();
    }

    public Mode getMode(){
        return mode;
    }

    @Override
    public void periodic() {
        switch (mode){
            case PID:
                front.set(0);
                pidf.setPIDF(kp, ki, kd, 0);
                pidf.setTolerance(positionTolerance);
                pidf.setSetPoint(tar);
                pwr = MathUtils.clamp(pidf.calculate(getCurrentPosition()) + kf, -1, 1);
                if(!pidf.atSetPoint()) back.set(pwr);
                else back.set(0);
                if(telemetryEnabled) t.addData("Lift Target Position", tar);
                break;
            case RAW:
                front.set(-pwr);
                back.set(pwr);
                break;
        }
        // Cache the clutch write: it only flips between two values, so don't re-send it every loop.
        double ptoTarget = (mode == Mode.INACTIVE) ? CLUTCH_PULLED : CLUTCH_MESHED;
        if(Double.isNaN(lastPto) || ptoTarget != lastPto){
            pto.set(ptoTarget);
            lastPto = ptoTarget;
        }

        if(telemetryEnabled){
            t.addData("Lift Current Position", getCurrentPosition());
            t.addData("Lift Power", pwr);
            t.addData("Lift Position Error", tar - getCurrentPosition());
            t.addData("Lift Reached Target?", reachedTarget());
            t.addData("Lift Mode", getMode().name());
        }
    }

    public boolean reachedTarget(){
        return pidf.atSetPoint();
    }

    public void resetEncoder(){
        back.getMotor().resetEncoder();
    }

    public Command SetModeCommand(Mode newMode){
        return new InstantCommand(() -> mode = newMode);
    }

    public Command FullLiftCommand(){
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        this.SetModeCommand(Mode.RAW),
                        new InstantCommand(() -> setManualPower(ENGAGING_POWER))
                ),
                new WaitCommand(ENGAGING_DELAY),
                new ParallelCommandGroup(
                        this.SetModeCommand(Mode.PID),
                        new InstantCommand(this::resetEncoder),
                        new InstantCommand(() -> setTargetPosition(LiftPositions.EXTENDED))
                ),
                new WaitUntilCommand(this::reachedTarget)
        );
    }
}
