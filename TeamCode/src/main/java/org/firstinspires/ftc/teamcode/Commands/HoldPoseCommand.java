package org.firstinspires.ftc.teamcode.Commands;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.Robot;

public class HoldPoseCommand extends CommandBase {

    private final Robot robot;
    private final GamepadEx driver;
    private final Pose targetPose; // the pose passed in (can be null)
    private Pose holdPose;         // resolved pose at initialize




    public HoldPoseCommand(Pose pose, GamepadEx driver) {
        this.robot = Robot.getInstance();
        this.driver = driver;
        this.targetPose = pose;    // use this specific pose
        addRequirements(robot.mecanumDrive);
    }

    @Override
    public void initialize() {
        holdPose = (targetPose != null)
                ? targetPose
                : robot.follower.getPose();

        robot.follower.followPath(
                robot.follower.pathBuilder()
                        .addPath(new BezierLine(holdPose, holdPose))
                        .setConstantHeadingInterpolation(holdPose.getHeading())
                        .build(),
                true
        );
    }


    @Override
    public void execute() {

//        robot.follower.update();
    }

    @Override
    public boolean isFinished() {
        // Only stop if holding current pose and driver moves sticks
//        if (targetPose == null) {
            return  Math.abs(driver.getLeftY()) > 0.1 ||
                    Math.abs(driver.getLeftX()) > 0.1 ||
                    Math.abs(driver.getRightX()) > 0.1;
//        } else {
//            return false; // holding a fixed pose ignores stick input
//        }
    }

    @Override
    public void end(boolean interrupted) {
        robot.follower.breakFollowing();
    }
}