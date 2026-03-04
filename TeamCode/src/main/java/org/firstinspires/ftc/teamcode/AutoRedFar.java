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
import org.firstinspires.ftc.teamcode.Commands.AlignToAprilTagCommand;
import org.firstinspires.ftc.teamcode.SubSystems.ColorMatch;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.SubSystems.Juggler;
import org.firstinspires.ftc.teamcode.SubSystems.JugglerAbsolute;

import java.util.Arrays;


//@Disabled
@Autonomous(name = "AutoRedFar", preselectTeleOp = "MainTeleOp", group = "Competition")
public class AutoRedFar extends CommandOpMode {
    Robot robot = Robot.getInstance();
    private ElapsedTime timer;
    private final Pose startPose = MirrorUtility.mirror(new Pose(61, 9, Math.toRadians(90)));
    private final Pose rotatedPose = MirrorUtility.mirror(new Pose(58, 10, Math.toRadians(116)));
    private final Pose artifactsGPPPose = MirrorUtility.mirror(new Pose(42, 35, Math.toRadians(180)));
    private final Pose collectGPPPose = MirrorUtility.mirror(new Pose(16, 35, Math.toRadians(180)));
    private final Pose shootFarPose = MirrorUtility.mirror(new Pose(56, 12, Math.toRadians(115)));
    private final Pose artifactPGPPose = MirrorUtility.mirror(new Pose(56, 57, Math.toRadians(180)));
    private final Pose collectPGPPose = MirrorUtility.mirror(new Pose(16, 57, Math.toRadians(180)));

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
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        endGPPToShootPath = robot.follower.pathBuilder()
                .addPath(new BezierLine(collectGPPPose, shootFarPose))
                .setConstantHeadingInterpolation(shootFarPose.getHeading())
//                .setLinearHeadingInterpolation(collectGPPPose.getHeading(), shootFarPose.getHeading())
                .build();

        shootToPGPSpikePath = robot.follower.pathBuilder()
                .addPath(new BezierLine(shootFarPose, artifactPGPPose))
                .setLinearHeadingInterpolation(shootFarPose.getHeading(), artifactPGPPose.getHeading())
                .build();

        eatPGPPath = robot.follower.pathBuilder()
                .addPath(new BezierLine(artifactPGPPose, collectPGPPose))
                .setLinearHeadingInterpolation(artifactPGPPose.getHeading(), collectPGPPose.getHeading())
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        endPGPToShootPath = robot.follower.pathBuilder()
                .addPath(new BezierLine(collectPGPPose, shootFarPose))
                .setConstantHeadingInterpolation(shootFarPose.getHeading())
                .build();
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
                        // Artifact detection & vision run in background parallel with all else
                        new DetectArtifactCommand(robot.rgbLight, robot.colorMatch, null),
                        new VisionCommand(robot.vision),

                        new SequentialCommandGroup(
                             new InstantCommand(()-> robot.vision.latchMotif()),
                                // Move to rotated shoot pose
                                new FollowPathCommand(robot.follower, rotateToShootPath, true),

                                // Shoot pre-loaded artifacts
                                new ShootCaseCommand(robot.juggler, robot.slide, robot.shooter, robot.colorMatch, robot.vision),
//                                new ShootLeftoversCommand(robot.juggler, robot.popper, robot.shooter, robot.colorMatch, robot.vision),
                                // Move to spike 1//
//                                new InstantCommand(()-> robot.shooter.setVelocity(2500)),
                                new FollowPathCommand(robot.follower, shootToGPPSpikePath, true, 1.0),


                                new ParallelDeadlineGroup(
                                        new IntakeCommand(robot.intake, Intake.MotorState.FORWARD, 2500, 0.75),
                                        new FollowPathCommand(robot.follower, eatGPPPath, true, 0.9),
                                        new SlowSpinPlusInterruptCommand(robot.juggler, JugglerAbsolute.Direction.CW)
                                ),

//                                new ParallelCommandGroup(
//                                        new IntakeCommand(robot.intake, Intake.MotorState.FORWARD, 2500, 0.75),
//                                        new FollowPathCommand(robot.follower, eatGPPPath, true, 0.9),
//                                        new RotateXSlotsCommand(robot.juggler, Juggler.Direction.CW, 2)
//
//                                ),

                                // Move to shoot position and stop intake
                                new ParallelCommandGroup(
                                        new InstantCommand(()->robot.juggler.snapToNearestSlot()),
                                        new FollowPathCommand(robot.follower, endGPPToShootPath, true, 1.0),
                                        new IntakeCommand(robot.intake, Intake.MotorState.FORWARD, 2500, 0.75),
                                        new InstantCommand(()->robot.juggler.snapToNearestSlot())
                                ),

                                // Shoot artifacts from spike 1
                                new ParallelCommandGroup(
                                        new InstantCommand(()-> robot.vision.latchMotif()),
                                        new InstantCommand(()->robot.juggler.snapToNearestSlot()),
                                        new InstantCommand(()->robot.intake.stop()),
                                        new ShootCaseCommand(robot.juggler, robot.slide, robot.shooter, robot.colorMatch, robot.vision)
//                                        new ShootLeftoversCommand(robot.juggler, robot.popper, robot.shooter, robot.colorMatch, robot.vision)
                                ),

                                // Move to spike 2
//                                new ShooterSpinUpCommand(robot.shooter,2500),
//                                new InstantCommand(()-> robot.shooter.setVelocity(2500)),

//                                new FollowPathCommand(robot.follower, shootToPGPSpikePath),
//
//                                // Collect balls on spike 2
//                                new ParallelDeadlineGroup(
//                                        new IntakeCommand(robot.intake, Intake.MotorState.FORWARD, 2500, 0.75),
//                                        new FollowPathCommand(robot.follower, eatPGPPath, true, 0.75),
//                                        new SlowSpinPlusInterruptCommand(robot.juggler, Juggler.Direction.CW)
//                                ),
//
//                                // Move to shoot position and stop intake
//                                new ParallelCommandGroup(
//                                        new FollowPathCommand(robot.follower, endPGPToShootPath, true, 1.0),
//                                        new InstantCommand(()->robot.intake.stop())
//                                ),
//
//                                // Shoot artifacts from spike 2
//                                new ShootCaseCommand(robot.juggler, robot.popper, robot.shooter, robot.colorMatch, robot.vision),
//
//                        // Park outside launch zone
                                new FollowPathCommand(robot.follower, shootToGPPSpikePath),
                                new InstantCommand(()->robot.shooter.stop())
                        )
                )
        );

        // INIT loop prior to coach pressing start
        while (opModeInInit()) {
            //robot.vision.scanForAprilTags();
            //robot.vision.latchMotif();
            if (robot.vision.getFirstSequence() != null) {
                robot.rgbLight.setColor(Color.AZURE);
            } else {
                robot.rgbLight.setColor(Color.RED);
            }
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
            //String temp = s0 + robot.colorMatch.getDistance(ColorMatch.Slot.SLOT_0);
            // Show slot colors
            telemetry.addData("Slot 0", s0);
            telemetry.addData("dist",robot.colorMatch.getDistance(ColorMatch.Slot.SLOT_0));
            telemetry.addData("h",robot.colorMatch.getHSV(ColorMatch.Slot.SLOT_0)[0]);
            telemetry.addData("s",robot.colorMatch.getHSV(ColorMatch.Slot.SLOT_0)[1]);

            telemetry.addData("Slot 1", s1);
            telemetry.addData("dist",robot.colorMatch.getDistance(ColorMatch.Slot.SLOT_1));
            telemetry.addData("h",robot.colorMatch.getHSV(ColorMatch.Slot.SLOT_1)[0]);
            telemetry.addData("s",robot.colorMatch.getHSV(ColorMatch.Slot.SLOT_1)[1]);

            telemetry.addData("Slot 2", s2);
            telemetry.addData("dist",robot.colorMatch.getDistance(ColorMatch.Slot.SLOT_2));
            telemetry.addData("h",robot.colorMatch.getHSV(ColorMatch.Slot.SLOT_2)[0]);
            telemetry.addData("s",robot.colorMatch.getHSV(ColorMatch.Slot.SLOT_2)[1]);

            telemetry.addLine("Hand-position artifacts on juggler");
            telemetry.addData("CurrentMotif: ", Arrays.toString(robot.vision.getFirstSequence()));
            telemetry.addData("LatchedMotif: ", Arrays.toString(robot.vision.getLatchedMotif()));
            telemetry.update();
        }


    }


//    @Override
//    public void run() {
//
//
//        super.run();
//        AprilTagDetection tag = robot.vision.getFirstTargetTag();
//        robot.follower.update();
//        robot.follower.getPose();
//        telemetry.addData("X:  ", robot.follower.getPose().getX());
//        telemetry.addData("Y:  ", robot.follower.getPose().getY());
//        telemetry.addData("Theta:  ", robot.follower.getPose().getHeading());
//        telemetry.addData("Slot 0", robot.colorMatch.detectColor(ColorMatch.Slot.SLOT_0));
//        telemetry.addData("Slot 1", robot.colorMatch.detectColor(ColorMatch.Slot.SLOT_1));
//        telemetry.addData("Slot 2", robot.colorMatch.detectColor(ColorMatch.Slot.SLOT_2));

//        telemetry.update();
//    }

    @Override
    public void run() {
        super.run();


        // AFTER coach presses START
        //AprilTagDetection tag = robot.vision.getFirstTargetTag();
        robot.follower.update();


        telemetry.addData("X", robot.follower.getPose().getX());
        telemetry.addData("Y", robot.follower.getPose().getY());
        telemetry.addData("Theta", robot.follower.getPose().getHeading());
        telemetry.addData("Slot 0", robot.colorMatch.detectColor(ColorMatch.Slot.SLOT_0));
        telemetry.addData("Slot 1", robot.colorMatch.detectColor(ColorMatch.Slot.SLOT_1));
        telemetry.addData("Slot 2", robot.colorMatch.detectColor(ColorMatch.Slot.SLOT_2));
        telemetry.update();
    }


    @Override
    public void end() {
        robot.autoEndPose = robot.follower.getPose();
    }


}

