package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Commands.DetectArtifactCommand;
import org.firstinspires.ftc.teamcode.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.ShootMotifCommand;
import org.firstinspires.ftc.teamcode.Commands.ShooterSpinUpCommand;
import org.firstinspires.ftc.teamcode.Commands.VisionCommand;
import org.firstinspires.ftc.teamcode.SubSystems.Gate;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


//@Disabled
@Autonomous(name="AutoBlueFar", preselectTeleOp = "MainTeleOp", group="Competition")
public class AutoBlueFar extends CommandOpMode {
    Robot robot = Robot.getInstance();

    private ElapsedTime timer;
    private final Pose startPose = new Pose(64, 9, Math.toRadians(90));
    private final Pose rotatedPose = new Pose(56, 12, Math.toRadians(117.5));
    private final Pose artifactsGPPPose = new Pose(56, 35.0, Math.toRadians(180));
    private final Pose collectGPPPose = new Pose(12, 35.0, Math.toRadians(180));
    private final Pose shootFarPose = new Pose(56, 12, Math.toRadians(117.5));
    private final Pose artifactPGPPose = new Pose(56, 60, Math.toRadians(180));
    private final Pose collectPGPPose = new Pose(12, 60, Math.toRadians(180));

    private PathChain rotateToShootPath, shootToGPPSpikePath, eatGPPPath, endGPPToShootPath, shootToPGPSpikePath,
            eatPGPPath, endPGPToShootPath;

    public void buildPaths() {
        robot.follower.setStartingPose(startPose);

        rotateToShootPath = robot.follower.pathBuilder()
                .addPath(new BezierLine(startPose, rotatedPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), rotatedPose.getHeading())
                .build();

        shootToGPPSpikePath = robot.follower.pathBuilder()
                .addPath(new BezierLine(rotatedPose, artifactsGPPPose))
                .setLinearHeadingInterpolation(rotatedPose.getHeading(), artifactsGPPPose.getHeading())
                .build();

        eatGPPPath = robot.follower.pathBuilder()
                .addPath(new BezierLine(artifactsGPPPose, collectGPPPose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        endGPPToShootPath= robot.follower.pathBuilder()
                .addPath(new BezierLine(collectGPPPose, shootFarPose))
                .setConstantHeadingInterpolation(Math.toRadians(-66))
                .build();

        shootToPGPSpikePath  =  robot.follower.pathBuilder()
                .addPath(new BezierLine(shootFarPose, artifactPGPPose))
                .setLinearHeadingInterpolation(shootFarPose.getHeading(), artifactPGPPose.getHeading())
                .build();

        eatPGPPath = robot.follower.pathBuilder()
                .addPath(new BezierLine(artifactPGPPose, collectPGPPose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        endPGPToShootPath= robot.follower.pathBuilder()
                .addPath(new BezierLine(collectPGPPose, shootFarPose))
                .setConstantHeadingInterpolation(Math.toRadians(117.7))
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
        robot.vision.setDefaultCommand(
                new VisionCommand(robot.vision)
        );

        robot.colorMatch.setDefaultCommand(
                new DetectArtifactCommand(robot.rgbLight, robot.colorMatch)
        );

        schedule(
                new SequentialCommandGroup(
                    new ParallelCommandGroup(
                    new ShooterSpinUpCommand(robot.shooter, 3850),
                    new FollowPathCommand(robot.follower, rotateToShootPath, true)
                    ),
                //shoot the three pre-loaded balls as a motif
                new ShootMotifCommand(robot.juggler, robot.popper, robot.colorMatch, robot.vision),

                //drive to first spike and shut down shooter
                new ParallelCommandGroup(
                    new FollowPathCommand(robot.follower, shootToGPPSpikePath, true),
                    new ShooterSpinUpCommand(robot.shooter,0.0)
                ),
                //gobble up all three balls on spike1
                new ParallelCommandGroup(
                    new IntakeCommand(robot.intake, Intake.MotorState.FORWARD,  2000),
                    new FollowPathCommand( robot.follower, eatGPPPath, true).setGlobalMaxPower(0.75)
                ),

                //move to shoot position, shut down intake, spin up shooter
                new ParallelCommandGroup(
                    new IntakeCommand(robot.intake, Intake.MotorState.STOP,  250),
                    new FollowPathCommand(robot.follower, endGPPToShootPath, true).setGlobalMaxPower(1.0),
                    new ShooterSpinUpCommand(robot.shooter, 3850)
                ),
                //shoot all three balls from spike1
                new ShootMotifCommand(robot.juggler, robot.popper, robot.colorMatch, robot.vision),

                //drive to spike2 and turn off shooter
                new ParallelCommandGroup(
                    new FollowPathCommand(robot.follower, shootToPGPSpikePath, true).setGlobalMaxPower(1.0),
                    new ShooterSpinUpCommand(robot.shooter,0.0)
                ),

                //gobble up spike2 balls
                new ParallelCommandGroup(
                    new IntakeCommand(robot.intake, Intake.MotorState.FORWARD,  2000),
                    new FollowPathCommand( robot.follower, eatPGPPath, true).setGlobalMaxPower(0.75)
                    ),

                //move to shoot position, shut down intake, spin up shooter
                new ParallelCommandGroup(
                    new IntakeCommand(robot.intake, Intake.MotorState.STOP,100),
                    new FollowPathCommand(robot.follower, endPGPToShootPath, true).setGlobalMaxPower(1.0),
                    new ShooterSpinUpCommand(robot.shooter,3850)
                    ),

                //shoot all three balls
                new ShootMotifCommand(robot.juggler, robot.popper, robot.colorMatch, robot.vision),

                //park outside of launch zone and power down subsystems
                new ParallelCommandGroup(
                   new FollowPathCommand(robot.follower, shootToGPPSpikePath, true).setGlobalMaxPower(1.0),
                   new ShooterSpinUpCommand(robot.shooter,0.0)
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
        telemetry.update();
    }

    @Override
    public void end() {
        robot.autoEndPose = robot.follower.getPose();
    }


}

