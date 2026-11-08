package org.firstinspires.ftc.teamcode.config.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

public class FastFollowPathCommand extends SequentialCommandGroup {
    public FastFollowPathCommand(Follower f, PathChain path){
        super(
                new InstantCommand(() -> f.followPath(path, true)),
                new WaitUntilCommand(() -> f.getCurrentTValue() > 0.94)
        );
    }
}
