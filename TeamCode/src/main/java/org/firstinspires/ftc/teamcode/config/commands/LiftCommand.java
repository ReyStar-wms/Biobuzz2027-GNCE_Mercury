package org.firstinspires.ftc.teamcode.config.commands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.config.subsystems.Intake;
import org.firstinspires.ftc.teamcode.config.subsystems.Lift;
import org.firstinspires.ftc.teamcode.config.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.config.subsystems.Turret;

public class LiftCommand extends SequentialCommandGroup {
    public LiftCommand(Intake intake, Turret turret, Shooter shooter, Lift lift){
        addCommands(
                new ParallelCommandGroup(
                        new InstantCommand(() -> intake.setMode(Intake.Mode.DISABLE)),
                        // new InstantCommand(() -> turret.) TODO: Disable
                        new InstantCommand(() -> shooter.setActive(false))
                ),
                lift.FullLiftCommand()
        );
        addRequirements(intake, turret, shooter, lift);
    }
}
