package org.firstinspires.ftc.teamcode.Cogintilities;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Robot;

public class PedroUtility {

    private PedroUtility(){}

    public static PathChain generateLinearBezierLinePath(Pose startPose, Pose endPose, Robot robot) {
        return robot.follower.pathBuilder()
                .addPath(new BezierLine(startPose, endPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
                .build();
    }
}