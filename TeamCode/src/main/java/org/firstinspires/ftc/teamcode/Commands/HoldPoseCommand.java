package org.firstinspires.ftc.teamcode.Commands;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.Robot;

public class HoldPoseCommand extends CommandBase {

    private final Robot robot;
    private final Pose holdPose;
    private final GamepadEx driver;

    public HoldPoseCommand(Pose holdPose, GamepadEx driver) {
        this.robot = Robot.getInstance();
        this.holdPose = holdPose;
        this.driver = driver;
        addRequirements(robot.mecanumDrive);
    }

    @Override
    public void initialize() {
        robot.follower.followPath(
                robot.follower.pathBuilder()
                        .addPath(new BezierLine(robot.follower.getPose(), holdPose))
                        .setLinearHeadingInterpolation(
                                robot.follower.getPose().getHeading(),
                                holdPose.getHeading()
                        )
                        .build(),
                true
        );
    }

    @Override
    public void execute() {

        robot.follower.update();
    }

    @Override
    public boolean isFinished() {
        // Driver override to end hold
        return Math.abs(driver.getLeftY()) > 0.1 ||
                Math.abs(driver.getLeftX()) > 0.1 ||
                Math.abs(driver.getRightX()) > 0.1;
    }

    @Override
    public void end(boolean interrupted) {
        robot.follower.breakFollowing();
    }
}
