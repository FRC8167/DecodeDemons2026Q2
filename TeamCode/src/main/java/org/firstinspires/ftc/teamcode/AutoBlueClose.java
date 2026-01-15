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
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Commands.DetectArtifactCommand;
import org.firstinspires.ftc.teamcode.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.ShooterSpinUpCommand;
import org.firstinspires.ftc.teamcode.Commands.VisionCommand;
import org.firstinspires.ftc.teamcode.SubSystems.Gate;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


//@Disabled
@Autonomous(name="AutoBlueClose" ,preselectTeleOp = "MainTeleOp", group="Competition")
public class AutoBlueClose extends CommandOpMode {
    Robot robot = Robot.getInstance();

    private ElapsedTime timer;
    private final Pose startPose = new Pose(26.5, 126.5, Math.toRadians(-45));
    private final Pose artifactsPPGPose = new Pose(56, 84, Math.toRadians(180));
    private final Pose collectPPGPose = new Pose(24, 84, Math.toRadians(180));
    private final Pose shootClosePose = new Pose(60, 78, Math.toRadians(-45));
    private final Pose artifactPGPPose = new Pose(56, 60, Math.toRadians(180));
    private final Pose collectPGPPose = new Pose(24, 60, Math.toRadians(180));



    private PathChain path1, path2, path3, path4, path5, path6, path7, path8;

    public void buildPaths() {
        robot.follower.setStartingPose(startPose);


        path1 = robot.follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootClosePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootClosePose.getHeading())
                .build();

        path2 = robot.follower.pathBuilder()
                .addPath(new BezierLine(shootClosePose, artifactsPPGPose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        path3= robot.follower.pathBuilder()
                .addPath(new BezierLine(artifactsPPGPose, collectPPGPose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        path4 = robot.follower.pathBuilder()
                .addPath(new BezierLine(collectPPGPose, shootClosePose))
                .setConstantHeadingInterpolation(Math.toRadians(-45))
                .build();
        path5 = robot.follower.pathBuilder()
                .addPath(new BezierLine(shootClosePose, collectPPGPose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        path6 = robot.follower.pathBuilder()
                .addPath(new BezierLine(shootClosePose, artifactPGPPose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        path7 = robot.follower.pathBuilder()
                .addPath(new BezierLine(artifactPGPPose, collectPGPPose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        path8 = robot.follower.pathBuilder()
                .addPath(new BezierLine(collectPGPPose ,shootClosePose))
                .setConstantHeadingInterpolation(Math.toRadians(-45))
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
                        new DetectArtifactCommand(robot.rgbLight, robot.colorMatch), // robot.shooter),
                        new VisionCommand(robot.vision),

                        new SequentialCommandGroup(
                                //move to launch zone
                                new ParallelCommandGroup(
                                        new FollowPathCommand(robot.follower, path1, true),
                                        new ShooterSpinUpCommand(robot.shooter, 3400)
                                ),
//                                new InstantCommand(() -> robot.vision.latchMotifFromTagIfEmpty()),

                                //shoot first artifact
                                new ParallelCommandGroup(
//                                        new FeederCommand(Feeder.FeederState.FORWARD, robot.feederR, 1000),
//                                        new FeederCommand(Feeder.FeederState.FORWARD, robot.feederF, 1000)
                                ),
                                new ShooterSpinUpCommand(robot.shooter, 3400),
                                //shoot second artifact
                                new ParallelCommandGroup(
//                                        new FeederCommand(Feeder.FeederState.FORWARD, robot.feederR, 1000),
//                                        new FeederCommand(Feeder.FeederState.FORWARD, robot.feederF, 1000),
                                        new IntakeCommand(robot.intake, Intake.MotorState.FORWARD,  1000)
                                ),
                                //move to nearest spike while shutting off intake, feeders, and shooter
                                new ParallelCommandGroup(
                                        new FollowPathCommand(robot.follower, path2, true),
//                                        new GateCommand(robot.gate, Gate.GateState.OPEN),
                                        new ShooterSpinUpCommand(robot.shooter,0.0),
//                                        new FeederCommand(Feeder.FeederState.STOP, robot.feederR, 100),
//                                        new FeederCommand(Feeder.FeederState.STOP, robot.feederF, 100),
                                        new IntakeCommand(robot.intake, Intake.MotorState.STOP,100)
                                ),
                                //gobble up artifacts on nearest spike
                                new ParallelCommandGroup(
//                                        new FeederCommand(Feeder.FeederState.FORWARD, robot.feederF, 1750),
                                        new IntakeCommand(robot.intake, Intake.MotorState.FORWARD,  1750),
                                        new FollowPathCommand( robot.follower, path3, true).setGlobalMaxPower(0.75)
                                ),

//                                        new GateCommand(robot.gate, Gate.GateState.CLOSED),

//                                new ParallelCommandGroup(
//                                        new FeederCommand(Feeder.FeederState.STOP, robot.feederF, 250),
                                        new IntakeCommand(robot.intake, Intake.MotorState.STOP,  250),
                                        new FollowPathCommand(robot.follower, path4, true).setGlobalMaxPower(1.0)

                                ),
                                //get ready to shoot
                                new ShooterSpinUpCommand(robot.shooter, 3400),
                                //shoot first artifact

                                new ParallelCommandGroup(
//                                        new FeederCommand(Feeder.FeederState.FORWARD, robot.feederR, 1000),
//                                        new FeederCommand(Feeder.FeederState.FORWARD, robot.feederF, 1000)
                                ),
                                new ShooterSpinUpCommand(robot.shooter, 3400),
                                //shoot second artifact
                                new ParallelCommandGroup(
//                                        new FeederCommand(Feeder.FeederState.FORWARD, robot.feederR, 3000),
//                                        new FeederCommand(Feeder.FeederState.FORWARD, robot.feederF, 3000),
                                        new IntakeCommand(robot.intake, Intake.MotorState.FORWARD,  3000)
                                ),
                                //move to middle spike and shut off systems
                                new ParallelCommandGroup(
                                        new FollowPathCommand(robot.follower, path6, true),
                                        new ShooterSpinUpCommand(robot.shooter, 0.0),
//                                        new FeederCommand(Feeder.FeederState.STOP, robot.feederR, 100),
//                                        new FeederCommand(Feeder.FeederState.STOP, robot.feederF, 100),
                                        new IntakeCommand(robot.intake, Intake.MotorState.STOP,  100)
//                                        new GateCommand(robot.gate, Gate.GateState.OPEN)
                                ),
                                //gobble up middle spike artifacts
                                new ParallelCommandGroup(
                                        new FollowPathCommand( robot.follower, path7, true),
//                                        new FeederCommand(Feeder.FeederState.FORWARD, robot.feederF, 2000),
                                        new IntakeCommand(robot.intake, Intake.MotorState.FORWARD,  2000)
                                ),
//                                new GateCommand(robot.gate, Gate.GateState.CLOSED),
                                //prepare and move to shoot position
                                new ParallelCommandGroup(

                                        new FollowPathCommand(robot.follower, path8 , true),
                                        new ShooterSpinUpCommand(robot.shooter, 3400),
//                                        new FeederCommand(Feeder.FeederState.STOP, robot.feederR, 100),
//                                        new FeederCommand(Feeder.FeederState.STOP, robot.feederF, 100),
                                        new IntakeCommand(robot.intake, Intake.MotorState.STOP,  100)

                                ),

                                new ParallelCommandGroup(
//                                        new FeederCommand(Feeder.FeederState.FORWARD, robot.feederR, 1000),
//                                        new FeederCommand(Feeder.FeederState.FORWARD, robot.feederF, 1000)
                                ),
                                new ShooterSpinUpCommand(robot.shooter, 3400),

                                //shoot second artifact
                                new ParallelCommandGroup(
//                                        new FeederCommand(Feeder.FeederState.FORWARD, robot.feederR, 3000),
//                                        new FeederCommand(Feeder.FeederState.FORWARD, robot.feederF, 3000),
                                        new IntakeCommand(robot.intake, Intake.MotorState.FORWARD,  3000)
                                ),
                                //park outside  of launch zone and power down systems
                                new ParallelCommandGroup(
//                                        new GateCommand(robot.gate, Gate.GateState.OPEN),
                                        new FollowPathCommand(robot.follower, path5, true),
                                        new ShooterSpinUpCommand(robot.shooter,0.0)
//                                        new FeederCommand(Feeder.FeederState.STOP, robot.feederR, 100),
//                                        new FeederCommand(Feeder.FeederState.STOP, robot.feederF, 100),
//                                        new IntakeCommand(robot.intake, Intake.MotorState.STOP, 100)
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

