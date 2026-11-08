package org.firstinspires.ftc.teamcode.config.core;

import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;

public abstract class MyCommandOpMode extends CommandOpMode {
    protected MyRobot r;

    public void atStart(){}

    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();
        initialize();

        // run the scheduler
        try {
            while (opModeInInit()) {
                r.startInitLoop();
                initialize_loop();
                r.endInitLoop();
            }
            atStart();
            r.onStart();
            while (!isStopRequested() && opModeIsActive()) {
                r.startPeriodic();
                run();
                r.endPeriodic();
            }
        } finally {
            try {
                r.stop();
                end();
            } finally {
                reset();
            }
        }
    }
}
