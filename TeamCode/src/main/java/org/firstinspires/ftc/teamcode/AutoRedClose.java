package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Cogintilities.Color;
import org.firstinspires.ftc.teamcode.Cogintilities.MirrorUtility;
import org.firstinspires.ftc.teamcode.Commands.DetectArtifactCommand;
import org.firstinspires.ftc.teamcode.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.Commands.ShootCaseCommand;
import org.firstinspires.ftc.teamcode.Commands.SlowSpinPlusInterruptCommand;
import org.firstinspires.ftc.teamcode.Commands.VisionCommand;
import org.firstinspires.ftc.teamcode.SubSystems.ColorMatch;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.SubSystems.Juggler;

import java.util.Arrays;


//@Disabled
@Autonomous(name="AutoRedClose" ,preselectTeleOp = "MainTeleOp", group="Competition")
public class AutoRedClose extends CommandOpMode {
    Robot robot = Robot.getInstance();

    private ElapsedTime timer;
    private final Pose startPose = MirrorUtility.mirror(new Pose(26.5, 126.5, Math.toRadians(135)));
    private final Pose latchPose = new Pose(88, 105, Math.toRadians(105));
//    private final Pose latchPose = MirrorUtility.mirror(new Pose(56, 110, Math.toRadians(80)));
    private final Pose artifactsPPGPose = MirrorUtility.mirror(new Pose(56, 84, Math.toRadians(180)));
    private final Pose collectPPGPose = MirrorUtility.mirror(new Pose(18, 84, Math.toRadians(180)));
    private final Pose shootClosePose = MirrorUtility.mirror(new Pose(56, 78, Math.toRadians(135)));
//    private final Pose artifactPGPPose = new Pose(56, 60, Math.toRadians(180));
//    private final Pose collectPGPPose = new Pose(24, 60, Math.toRadians(180));



    private PathChain startToLatchPath, latchToShootClosePath, shootCloseToSpike1Path, collectPPGArtifactsPath, spike1ToShootPath, parkPath;

    public void buildPaths() {
        robot.follower.setStartingPose(startPose);


        startToLatchPath = robot.follower.pathBuilder()
                .addPath(new BezierLine(startPose, latchPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), latchPose.getHeading())
                .build();

        latchToShootClosePath = robot.follower.pathBuilder()
                .addPath(new BezierLine(latchPose, shootClosePose))
                .setLinearHeadingInterpolation(latchPose.getHeading(), shootClosePose.getHeading())
                .build();

        shootCloseToSpike1Path = robot.follower.pathBuilder()
                .addPath(new BezierLine(shootClosePose, artifactsPPGPose))
                .setLinearHeadingInterpolation(shootClosePose.getHeading(), artifactsPPGPose.getHeading())
                .build();

        collectPPGArtifactsPath= robot.follower.pathBuilder()
                .addPath(new BezierLine(artifactsPPGPose, collectPPGPose))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        spike1ToShootPath = robot.follower.pathBuilder()
                .addPath(new BezierLine(collectPPGPose, shootClosePose))
                .setConstantHeadingInterpolation(Math.toRadians(45))
                .build();
        parkPath = robot.follower.pathBuilder()
                .addPath(new BezierLine(shootClosePose, collectPPGPose))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
//        path5 = robot.follower.pathBuilder()
//                .addPath(new BezierLine(shootClosePose, collectPPGPose))
//                .setConstantHeadingInterpolation(Math.toRadians(180))
//                .build();
//        path6 = robot.follower.pathBuilder()
//                .addPath(new BezierLine(shootClosePose, artifactPGPPose))
//                .setConstantHeadingInterpolation(Math.toRadians(180))
//                .build();
//        path7 = robot.follower.pathBuilder()
//                .addPath(new BezierLine(artifactPGPPose, collectPGPPose))
//                .setConstantHeadingInterpolation(Math.toRadians(180))
//                .build();
//        path8 = robot.follower.pathBuilder()
//                .addPath(new BezierLine(collectPGPPose ,shootClosePose))
//                .setConstantHeadingInterpolation(Math.toRadians(-45))
//                .build();
    }

    public void initialize() {
        Robot.OP_MODE_TYPE = Robot.OpModeType.AUTO;
        robot.setAlliance(Robot.Alliance.RED);

        timer = new ElapsedTime();
        timer.reset();

        // DO NOT REMOVE! Resetting FTCLib Command Scheduler
        super.reset();

        try {
            robot.init(hardwareMap, true);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        buildPaths();

        schedule(
                new ParallelCommandGroup(
                        new DetectArtifactCommand(robot.rgbLight, robot.colorMatch, null), // robot.shooter),
                        new VisionCommand(robot.vision),

                        new SequentialCommandGroup(
                                //move to latch
                                new FollowPathCommand(robot.follower, startToLatchPath, true),

                                //latch
//                                new InstantCommand(()-> robot.vision.latchMotif()),

                                //move to launch zone
                                new FollowPathCommand(robot.follower, latchToShootClosePath),

                                //shoot pre-loaded artifacts
                                new ShootCaseCommand(robot.juggler, robot.slide, robot.shooter, robot.colorMatch, robot.vision),
                                //new ShootLeftoversCommand(robot.juggler, robot.popper, robot.shooter, robot.colorMatch, robot.vision),
                                //move to spike 1
                                new InstantCommand(()-> robot.shooter.setVelocity(2500)),
                                new FollowPathCommand(robot.follower, shootCloseToSpike1Path, true, 1.0),

                                new ParallelDeadlineGroup(
                                        new IntakeCommand(robot.intake, Intake.MotorState.FORWARD, 2500, 0.75),
                                        new FollowPathCommand(robot.follower, collectPPGArtifactsPath, true, 0.9),
                                        new SlowSpinPlusInterruptCommand(robot.juggler, Juggler.Direction.CW)
                                ),
                                //Move to shoot position and stop intake
                                new ParallelCommandGroup(
                                        new FollowPathCommand(robot.follower, spike1ToShootPath, true, 1.0),
                                        new IntakeCommand(robot.intake, Intake.MotorState.FORWARD, 2500, 0.75),
                                        new InstantCommand(()->robot.juggler.snapToNearestSlot())
                                ),

                                //shoot artifacts from spike1
                                new ParallelCommandGroup(
                                        new ShootCaseCommand(robot.juggler, robot.slide, robot.shooter, robot.colorMatch, robot.vision),
                                        new InstantCommand(()->robot.intake.stop())
                                        //new ShootLeftoversCommand(robot.juggler, robot.popper, robot.shooter, robot.colorMatch, robot.vision)
                                ),
                                //park outside launch zone
                                new FollowPathCommand(robot.follower, parkPath),
                                new InstantCommand(()->robot.shooter.stop())

                        )
                )
        );


        // INIT loop prior to coach pressing start
        while (opModeInInit()) {
//            robot.vision.scanForAprilTags();
//            robot.vision.latchMotif();
//            if (robot.vision.getMotifPattern() != null) {
//                robot.rgbLight.setColor(Color.AZURE);
//            } else {
//                robot.rgbLight.setColor(Color.RED);
//            }
            // Detect the slots
            ColorMatch.ArtifactColor s0 = robot.colorMatch.detectColor(ColorMatch.Slot.SLOT_0);
            ColorMatch.ArtifactColor s1 = robot.colorMatch.detectColor(ColorMatch.Slot.SLOT_1);
            ColorMatch.ArtifactColor s2 = robot.colorMatch.detectColor(ColorMatch.Slot.SLOT_2);

            // Check if all slots are known
            boolean ready = s0 != ColorMatch.ArtifactColor.UNKNOWN &&
                    s1 != ColorMatch.ArtifactColor.UNKNOWN &&
                    s2 != ColorMatch.ArtifactColor.UNKNOWN;

            // Telemetry banner
            telemetry.addLine("==============================");
            telemetry.addLine(ready
                    ? "     ✅ READY TO START AUTO"
                    : "     ❌ REPOSITION ARTIFACTS"
            );
            telemetry.addLine("==============================");

            // Show slot colors
            telemetry.addData("Slot 0", s0);
            telemetry.addData("Slot 1", s1);
            telemetry.addData("Slot 2", s2);

            telemetry.addLine("Hand-position artifacts on juggler");
            telemetry.addData("CurrentMotif: ", Arrays.toString(robot.vision.getFirstSequence()));
            telemetry.addData("LatchedMotif: ", Arrays.toString(robot.vision.getLatchedMotif()));
            telemetry.update();
        }



    }




    @Override
    public void run() {
        super.run();
        //AprilTagDetection tag = robot.vision.getFirstTargetTag();
//        LLResultTypes.FiducialResult tag = robot.vision.getAprilTags().get(0);
        if (robot.vision.getFirstSequence() != null) {
            robot.rgbLight.setColor(Color.AZURE);
        } else {
            robot.rgbLight.setColor(Color.RED);
        }
        robot.follower.update();
        robot.follower.getPose();
        telemetry.addData("X:  ", robot.follower.getPose().getX());
        telemetry.addData("Y:  ", robot.follower.getPose().getY());
        telemetry.addData("Theta:  ", robot.follower.getPose().getHeading());
//        if (tag != null) {
//            telemetry.addLine("Target Tag Detected!");
//            telemetry.addData("ID", tag.getFiducialId());
//            Position targetInCamera = tag.getTargetPoseCameraSpace().getPosition();
//            telemetry.addData("Center", "(%.0f, %.0f)", targetInCamera.x, targetInCamera.z); //Note: These values may be swapped
//            telemetry.addData("Range (in)", "%.1f", Math.sqrt(Math.pow(targetInCamera.x, 2) + Math.pow(targetInCamera.y, 2) + Math.pow(targetInCamera.z, 2)));
//
//        } else {
//            telemetry.addLine("No target tags (20–24) detected.");
//        }
        telemetry.update();
    }

    @Override
    public void end() {
        robot.autoEndPose = robot.follower.getPose();
    }


}

