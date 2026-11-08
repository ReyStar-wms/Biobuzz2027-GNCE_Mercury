package org.firstinspires.ftc.teamcode.config.subsystems;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.PerpetualCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config.core.SubsysCore;
import org.firstinspires.ftc.teamcode.config.hardware.CachedServo;

@Configurable
public class Door extends SubsysCore {
    CachedServo door;
    public static double OFFSET_POS = 0;
    public static double OPEN_POS = 0.22;
    public static double CLOSED_POS = 0.39;
    public static long ACTUATION_TIME_MS = 180;

    public static long OPEN_DELAY_MS = 150;   // delay between detecting 3 balls and opening
    private long fullStartMs = -1;             // when size first became 3 (-1 = not full)

    boolean open;

    public Door() {
        door = new CachedServo(h.get(Servo.class, "door"));
        open = false;
        setDefaultCommand(new RunCommand(() -> {
            if (size == 3) {
                if (fullStartMs < 0) fullStartMs = System.currentTimeMillis();
                setOpen(System.currentTimeMillis() - fullStartMs >= OPEN_DELAY_MS);
            } else {
                fullStartMs = -1;
                setOpen(false);
            }
        }, this));
    }

    int size;

    public void inputSize(int size){
        this.size = size;
    }

    public boolean isOpen(){ return open; }

    public void setOpen(boolean state){ open = state; }


    public Command setOpenCommand(boolean state){
        return new ConditionalCommand(
                new WaitCommand(ACTUATION_TIME_MS),
                new InstantCommand(),
                () -> {
                    boolean changed = (open != state);
                    open = state;
                    return changed;
                }
        );
    }

    @Override
    public void periodic() {
        door.setPosition(OFFSET_POS + (open ? OPEN_POS : CLOSED_POS));
        if(telemetryEnabled) t.addData("Door State", open ? "Open" : "Closed");
    }
}
