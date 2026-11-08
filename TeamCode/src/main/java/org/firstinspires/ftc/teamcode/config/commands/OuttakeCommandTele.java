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

public class OuttakeCommandTele extends SequentialCommandGroup {
    public OuttakeCommandTele(Intake intake, Turret turret, Shooter shooter, Door door, Storage storage){
        this(intake, turret, shooter, door, storage, null);
    }

    /**
     * @param onLaunch fired the instant the transfer starts pushing the balls out -- the reference
     *                 point the rumble cue's timing is computed from. May be null.
     */
    public OuttakeCommandTele(Intake intake, Turret turret, Shooter shooter, Door door, Storage storage,
                              Runnable onLaunch){
        addCommands(
                new WaitUntilCommand(shooter::readyToShoot),
                door.setOpenCommand(true),
                new InstantCommand(() -> {
                    intake.setMode(Intake.Mode.TRANSFER);
                    if (onLaunch != null) onLaunch.run(); // transfer push = balls about to clear: cue timing anchor
                }),
                new WaitCommand(200),
                new ParallelCommandGroup(
                        new InstantCommand(() -> intake.setMode(Intake.Mode.INTAKE)),
                        new InstantCommand(storage::clear)
                )
        );
        addRequirements(door, shooter, intake, turret);
    }
}
