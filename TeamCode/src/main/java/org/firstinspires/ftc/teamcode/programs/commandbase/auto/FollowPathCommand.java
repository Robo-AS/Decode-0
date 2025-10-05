package org.firstinspires.ftc.teamcode.programs.commandbase.auto;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;

public class FollowPathCommand extends CommandBase {
    private final Follower follower;
    private final PathChain path;
    private final boolean resetPose;

    public FollowPathCommand(Follower follower, PathChain path, boolean resetPose) {
        this.follower = follower;
        this.path = path;
        this.resetPose = resetPose;
    }

    @Override
    public void initialize() {
        follower.followPath(path, resetPose);
    }

    @Override
    public void execute() {
        follower.update();
    }

    @Override
    public boolean isFinished() {
        return !follower.isBusy();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            follower.breakFollowing();
        }
    }
}

