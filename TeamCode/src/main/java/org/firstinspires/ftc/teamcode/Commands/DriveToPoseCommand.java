package org.firstinspires.ftc.teamcode.Commands;


import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Robot;


public class DriveToPoseCommand extends CommandBase {

    private final Pose targetPose; // target pose including heading
    private final Robot robot;
    private final GamepadEx driver;



    public DriveToPoseCommand(Pose targetPose, GamepadEx driver)
    {
        robot = Robot.getInstance();
        this.targetPose = targetPose;
        this.driver = driver;
//        addRequirements();  //TODO check if this works

    }

    @Override
    public void initialize() {

        Pose currentPose = robot.follower.getPose();

        PathChain pathToShoot = robot.follower.pathBuilder()
                .addPath(new BezierLine(currentPose, targetPose))
                .setLinearHeadingInterpolation(
                        currentPose.getHeading(),
                        targetPose.getHeading()
                )
                .build();
//        new FollowPathCommand(robot.follower, pathToShoot, true);
        robot.follower.followPath(pathToShoot, true);

    }


    @Override
    public void execute() {
        robot.follower.update();
    }


    @Override
    public boolean isFinished() {

        boolean driverOverride =
                Math.abs(driver.getLeftY()) > 0.1 ||
                        Math.abs(driver.getLeftX()) > 0.1 ||
                        Math.abs(driver.getRightX()) > 0.1;

        return !robot.follower.isBusy() || driverOverride;
    }


    @Override
    public void end(boolean interrupted) {
        robot.follower.breakFollowing();
    }





}
