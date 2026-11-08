package org.firstinspires.ftc.teamcode.config.commands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.config.subsystems.Door;
import org.firstinspires.ftc.teamcode.config.subsystems.Intake;
import org.firstinspires.ftc.teamcode.config.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.config.subsystems.Storage;
import org.firstinspires.ftc.teamcode.config.subsystems.Turret;

public class TransferCommand extends SequentialCommandGroup {
    public TransferCommand(Intake intake,  Storage storage){
        addCommands(
                new InstantCommand(() -> intake.setMode(Intake.Mode.TRANSFER)),
                new WaitUntilCommand(() -> storage.getSize() == 0).withTimeout(1000),
                new InstantCommand(() -> intake.setMode(Intake.Mode.INTAKE))
        );
        addRequirements(intake);
    }
}
