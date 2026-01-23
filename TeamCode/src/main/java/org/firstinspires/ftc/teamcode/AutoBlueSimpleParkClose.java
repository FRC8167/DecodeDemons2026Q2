package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Commands.DetectArtifactCommand;
import org.firstinspires.ftc.teamcode.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.ShooterSpinUpCommand;
import org.firstinspires.ftc.teamcode.Commands.VisionCommand;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


//@Disabled
@Autonomous(name="AutoBlueSimpleParkClose" ,preselectTeleOp = "MainTeleOp", group="Competition")
public class AutoBlueSimpleParkClose extends CommandOpMode {
    Robot robot = Robot.getInstance();

    private ElapsedTime timer;
    private final Pose startPose = new Pose(26.5, 126.5, Math.toRadians(-45));
    private final Pose shootClosePose = new Pose(60, 78, Math.toRadians(-45));
    private final Pose endParkPose = new Pose(22, 108, Math.toRadians(-45));




    private PathChain path1, shootToParkPose;

    public void buildPaths() {
        robot.follower.setStartingPose(startPose);


        path1 = robot.follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootClosePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootClosePose.getHeading())
                .build();
        shootToParkPose = robot.follower.pathBuilder()
                .addPath(new BezierLine(shootClosePose, endParkPose))
                .setLinearHeadingInterpolation(shootClosePose.getHeading(), endParkPose.getHeading())
                .build();


    }

    public void initialize() {
        Robot.OP_MODE_TYPE = Robot.OpModeType.AUTO;
        robot.setAlliance(Robot.Alliance.BLUE);

        timer = new ElapsedTime();
        timer.reset();

        // DO NOT REMOVE! Resetting FTCLib Command Scheduler
        super.reset();

        try {
            robot.init(hardwareMap);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        buildPaths();

        schedule(
                new ParallelCommandGroup(
                        new DetectArtifactCommand(robot.rgbLight, robot.colorMatch, null), // robot.shooter),
                        new VisionCommand(robot.vision),
                new SequentialCommandGroup(
                        //move to launch zone
                        new ParallelCommandGroup(
                                new FollowPathCommand(robot.follower, path1, true),
                                new ShooterSpinUpCommand(robot.shooter, 3400)
                        ),
                        //shoot first artifact
                        new ParallelCommandGroup(
//                                new FeederCommand(Feeder.FeederState.FORWARD, robot.feederR, 1000),
//                                new FeederCommand(Feeder.FeederState.FORWARD, robot.feederF, 1000)
                        ),
                        new WaitCommand(250),
                        //shoot second artifact
                        new ParallelCommandGroup(
//                                new FeederCommand(Feeder.FeederState.FORWARD, robot.feederR, 1000),
//                                new FeederCommand(Feeder.FeederState.FORWARD, robot.feederF, 1000),
                                new IntakeCommand(robot.intake, Intake.MotorState.FORWARD,  1000, 0.5)
                        ),
                        //park and power down systems
                        new ParallelCommandGroup(
                                new FollowPathCommand(robot.follower, shootToParkPose, true),
                                new ShooterSpinUpCommand(robot.shooter,0.0),
//                                new FeederCommand(Feeder.FeederState.STOP, robot.feederR, 100),
//                                new FeederCommand(Feeder.FeederState.STOP, robot.feederF, 100),
                                new InstantCommand(()->robot.intake.stop())
                                )

                        )

                )

        );

    }




    @Override
    public void run() {
        super.run();
        AprilTagDetection tag = robot.vision.getFirstTargetTag();
        robot.follower.update();
        robot.follower.getPose();
        telemetry.addData("X:  ", robot.follower.getPose().getX());
        telemetry.addData("Y:  ", robot.follower.getPose().getY());
        telemetry.addData("Theta:  ", robot.follower.getPose().getHeading());
        if (tag != null) {
            telemetry.addLine("Target Tag Detected!");
            telemetry.addData("ID", tag.id);
            telemetry.addData("Center", "(%.0f, %.0f)", tag.center.x, tag.center.y);
            telemetry.addData("Range (in)", "%.1f", tag.ftcPose.range);
        } else {
            telemetry.addLine("No target tags (20–24) detected.");
        }
        telemetry.update();
    }

    @Override
    public void end() {
        robot.autoEndPose = robot.follower.getPose();
    }


}

