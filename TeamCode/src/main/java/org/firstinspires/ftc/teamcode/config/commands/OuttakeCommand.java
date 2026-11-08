package org.firstinspires.ftc.teamcode.config.commands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.config.subsystems.Door;
import org.firstinspires.ftc.teamcode.config.subsystems.Intake;
import org.firstinspires.ftc.teamcode.config.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.config.subsystems.Storage;
import org.firstinspires.ftc.teamcode.config.subsystems.Turret;

public class OuttakeCommand extends SequentialCommandGroup {
    public OuttakeCommand(Intake intake, Turret turret, Shooter shooter, Door door, Storage storage){
        addCommands(
                new ParallelCommandGroup(
                        //new WaitUntilCommand(turret::reachedTarget),
                        new WaitUntilCommand(shooter::readyToShoot)
                ),
                door.setOpenCommand(true),
                new InstantCommand(() -> intake.setMode(Intake.Mode.TRANSFER)),
                new WaitUntilCommand(() -> storage.getSize() == 0).withTimeout(1000),
                new WaitCommand(400),
                new WaitUntilCommand(() -> storage.getSize() == 0).withTimeout(500),
                new InstantCommand(() -> intake.setMode(Intake.Mode.INTAKE))
        );
        addRequirements(door, shooter, intake, turret);
    }
}
