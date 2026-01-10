package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
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
@Autonomous(name="AutoBlueSimpleParkFar", preselectTeleOp = "MainTeleOp", group="Autonomous")
public class AutoBlueSimpleParkFar extends CommandOpMode {
    Robot robot = Robot.getInstance();

    private ElapsedTime timer;
    private final Pose startPose = new Pose(64, 9,Math.toRadians(-90));
    private final Pose rotatedPose =(new Pose(56, 12, Math.toRadians(-66)));
    private final Pose parkPose = (new Pose(36, 12, Math.toRadians(-90)));



    private PathChain rotateToShootPath, shootToParkPath;

    public void buildPaths() {
        robot.follower.setStartingPose(startPose);


        rotateToShootPath = robot.follower.pathBuilder()
                .addPath(new BezierLine(startPose, rotatedPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), rotatedPose.getHeading())
                .build();

        shootToParkPath = robot.follower.pathBuilder()
                .addPath(new BezierLine(rotatedPose, parkPose))
                .setLinearHeadingInterpolation(rotatedPose.getHeading(), parkPose.getHeading())
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
                        new DetectArtifactCommand(robot.rgbLight, robot.colorMatch),// robot.shooter),
                        new VisionCommand(robot.vision),

                        new SequentialCommandGroup(

                                //rotate and get ready to shoot
                                new ParallelCommandGroup(
                                        new ShooterSpinUpCommand(robot.shooter, 3850),
                                        new FollowPathCommand(robot.follower, rotateToShootPath, true)
                                ),
                                //shoot first ball
                                new ParallelCommandGroup(
//                                        new FeederCommand(Feeder.FeederState.FORWARD, robot.feederR, 1000),
//                                        new FeederCommand(Feeder.FeederState.FORWARD, robot.feederF, 1000)
                                ),
                                new WaitCommand(250),
                                //shoot second ball
                                new ParallelCommandGroup(
//                                        new FeederCommand(Feeder.FeederState.FORWARD, robot.feederR, 1000),
//                                        new FeederCommand(Feeder.FeederState.FORWARD, robot.feederF, 1000),
                                        new IntakeCommand(robot.intake, Intake.MotorState.FORWARD,  1000)
                                ),
                                //move to park
                                new ParallelCommandGroup(
                                        new FollowPathCommand(robot.follower, shootToParkPath, true),
                                        new ShooterSpinUpCommand(robot.shooter,0.0),
//                                        new FeederCommand(Feeder.FeederState.STOP, robot.feederR, 250),
//                                        new FeederCommand(Feeder.FeederState.STOP, robot.feederF, 250),
                                        new IntakeCommand(robot.intake, Intake.MotorState.STOP,250)
                                )

                        )
                )
        );

        //park somewhere
//                                new ParallelCommandGroup(
//
//                                        new FeederCommand(Feeder.FeederState.FORWARD, robot.feederF, 2000),
//                                        new FeederCommand(Feeder.FeederState.FORWARD, robot.feederR, 2000),
//                                        new IntakeCommand(robot.intake, Intake.MotorState.FORWARD,  2000),
//                                        new FollowPathCommand( robot.follower, path6, true)
//                                ),
//
//
//                                new ParallelCommandGroup(
//                                        new ShooterSpinUpCommand(robot.shooter,4200),
//
//                                        new FeederCommand(Feeder.FeederState.FORWARD, robot.feederR, 3000),
//                                        new FeederCommand(Feeder.FeederState.FORWARD, robot.feederF, 3000),
//                                        new IntakeCommand(robot.intake, Intake.MotorState.FORWARD,  3000)


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
        telemetry.update();
    }

    @Override
    public void end() {
        robot.autoEndPose = robot.follower.getPose();
    }


}

