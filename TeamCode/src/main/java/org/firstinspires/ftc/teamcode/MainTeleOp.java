package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Commands.CancelPedroCommand;
import org.firstinspires.ftc.teamcode.Commands.DetectArtifactCommand;
import org.firstinspires.ftc.teamcode.Commands.DriveCommand;
import org.firstinspires.ftc.teamcode.Commands.DriveToPoseCommand;
import org.firstinspires.ftc.teamcode.Commands.HoldPoseCommand;
import org.firstinspires.ftc.teamcode.Commands.RotateOneSlotCommand;
import org.firstinspires.ftc.teamcode.Commands.ShootMotifCommand;
import org.firstinspires.ftc.teamcode.Commands.ShooterSmartSpinUpCommand;
import org.firstinspires.ftc.teamcode.Commands.ShooterSpinUpCommand;
import org.firstinspires.ftc.teamcode.Commands.VisionCommand;
import org.firstinspires.ftc.teamcode.SubSystems.ColorMatch;
import org.firstinspires.ftc.teamcode.SubSystems.Juggler;
import org.firstinspires.ftc.teamcode.SubSystems.Popper;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Configurable
//@Disabled
@TeleOp(name="MainTeleOp", group="Competition")
public class MainTeleOp extends CommandOpMode {

    public GamepadEx driver;
    public GamepadEx operator;
    public ElapsedTime timer;
    private final Robot robot = Robot.getInstance();
    static TelemetryManager telemetryM;
    Pose currentPose;
    private ShootMotifCommand shootMotifCommand;



    private Pose startPose;
//    private Pose autoEndPose;



    @Override
    public void initialize() {

        // Must have for all opModes
        Robot.OP_MODE_TYPE = Robot.OpModeType.TELEOP;
        // Resets the command scheduler
        super.reset();
        //Initialize the robot
        try {
            robot.init(hardwareMap);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        if (robot.autoEndPose == null) {
            startPose = new Pose(0, 0, 0);
        } else {
            startPose = robot.autoEndPose;
        }
        //added 12-22
        robot.mecanumDrive.setDefaultCommand(new DriveCommand(robot.mecanumDrive, gamepad1));
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        //end pose held in robot
        //if auto ran, last pose is used or else default of 24,24,0
        Pose startPose = robot.autoEndPose != null ? robot.autoEndPose : new Pose(24, 24, 0);
        robot.follower.setStartingPose(startPose);
        robot.follower.update();
        //only schedule perpetually running commands
        schedule(new DriveCommand(robot.mecanumDrive, gamepad1));
        schedule(new VisionCommand(robot.vision));
        schedule(new DetectArtifactCommand(robot.rgbLight, robot.colorMatch)); //, robot.shooter));

        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);


        //******OPERATOR CONTROLS*****
        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(new ShooterSmartSpinUpCommand(robot.shooter, robot.vision))
                .whenReleased(new ShooterSpinUpCommand(robot.shooter, 0.0));


//        operator.getGamepadButton(GamepadKeys.Button.BACK)
//                .whileHeld(new ShooterSpinUpCommand(robot.shooter, -1000.0))
//                .whenReleased(new ShooterSpinUpCommand(robot.shooter, 0.0));


        operator.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(robot.intake::forward));

        operator.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(robot.intake::reverse));

        operator.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).
                whenPressed(new RotateOneSlotCommand(robot.juggler, Juggler.Direction.CW));

        operator.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).
                whenPressed(new RotateOneSlotCommand(robot.juggler, Juggler.Direction.CCW));

        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(
                        new SequentialCommandGroup(
                                new InstantCommand(()->robot.popper.set(Popper.PopperState.KICK)),
                                new WaitCommand(500),
                                new InstantCommand(()->robot.popper.set(Popper.PopperState.RESET))
                        )
                );

        shootMotifCommand = new ShootMotifCommand(
                robot.juggler,
                robot.popper,
                robot.colorMatch,
                robot.vision
        );

        operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(() -> {
                    robot.vision.latchMotifFromTagIfEmpty();
                }));


        operator.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(shootMotifCommand);

        operator.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(shootMotifCommand::skipNext));









        /* ******************************************************************************* */
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(robot.mecanumDrive::enableSnailDrive))
                .whenReleased(new InstantCommand(robot.mecanumDrive::disableSnailDrive));

        //must be HELD DOWN to hold the position
        //operator can still use anything
        //once driver releases and moves joystick, the hold ends
        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(new HoldPoseCommand(robot.follower.getPose(), driver));

        driver.getGamepadButton(GamepadKeys.Button.BACK)
                .whenPressed(new CancelPedroCommand());


        driver.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(
                        new SequentialCommandGroup(
                                //Spin up shooter while driving to far shoot pose
                                new ParallelCommandGroup(
                                        new ShooterSpinUpCommand(robot.shooter, 3850),
                                        new DriveToPoseCommand(robot.getShootPose(), driver)
                                ),
                                //Hold the pose as defensive strategy
                                new HoldPoseCommand(robot.getShootPose(), driver),
                                //Motif shoot command
                                new ShootMotifCommand(robot.juggler, robot.popper, robot.colorMatch, robot.vision),
                                //Spin down shooter
                                new ShooterSpinUpCommand(robot.shooter, 0),
                                //End the path hold
                                new InstantCommand(() -> robot.follower.breakFollowing())
                        )
                );

    }

    @Override
    public void run() {
        super.run();
        robot.follower.update();
        robot.autoEndPose = robot.follower.getPose();
        AprilTagDetection tag = robot.vision.getFirstTargetTag();
        telemetry.addData("ShootMotif", shootMotifCommand.getStatus());


        if (tag != null) {
            telemetry.addLine("Target Tag Detected!");
            telemetry.addData("ID", tag.id);
            telemetry.addData("Center", "(%.0f, %.0f)", tag.center.x, tag.center.y);
            telemetry.addData("Range (in)", "%.1f", tag.ftcPose.range);
        } else {
            telemetry.addLine("No target tags (20–24) detected.");
        }


//       if (gamepad2.right_trigger > 0.3) {
//           robot.feederR.feed(Feeder.FeederState.FORWARD);
//           robot.feederF.feed(Feeder.FeederState.FORWARD);
//
//       }else{
//           robot.feederR.feed(Feeder.FeederState.STOP);
//           robot.feederF.feed(Feeder.FeederState.STOP);
//       }


        telemetry.addData("autoEndPose", robot.autoEndPose.toString());
        telemetry.addData("FollowerX", Math.round(robot.follower.getPose().getX() * 100) / 100.0);
        telemetry.addData("FollowerY", Math.round(robot.follower.getPose().getY() * 100) / 100.0);
        telemetry.addData("FollowerH", Math.round(Math.toDegrees(robot.follower.getPose().getHeading()) * 100) / 100.0);
        telemetry.addData("Distance to Goal", robot.vision.getDistanceToGoal());
        telemetryM.addData("Obelisk Motif", robot.vision.getMotifPatternString());


        telemetry.addData("Shooter Velocity (RPM)", robot.shooter.getRPM());
        telemetry.addData("Shooter Ready?", robot.shooter.atTargetVelocity());
        telemetry.addData("Juggler counts", robot.juggler.getCurrentPosition());



        telemetryM.addData("Slot 0", robot.colorMatch.detectColor(ColorMatch.Slot.SLOT_0));
        telemetryM.addData("Slot 1", robot.colorMatch.detectColor(ColorMatch.Slot.SLOT_1));
        telemetryM.addData("Slot 2", robot.colorMatch.detectColor(ColorMatch.Slot.SLOT_2));
//        telemetry.update();
        telemetryM.addData("Shooter Ready?", robot.shooter.atTargetVelocity());
        

//        telemetry.update();
        telemetryM.update(telemetry);

    }


    @Override
    public void end() {
        robot.autoEndPose = robot.follower.getPose();
    }

    public Pose getAutoEndPose() {
        return robot.autoEndPose;
    }


//    PathChain createDrivePath(Pose targetPose) {
//
//        currentPose =robot.follower.getPose();
//
////        endPGPToShootPath= robot.follower.pathBuilder()
////                .addPath(new BezierLine(collectPGPPose, shootFarPose))
////                .setConstantHeadingInterpolation(Math.toRadians(-66))
////                .build();
//
//        PathChain pathToShoot = robot.follower.pathBuilder()
//                .addPath(new BezierLine(currentPose, robot.getShootPose()))
//                .setConstantHeadingInterpolation(Math.toRadians(robot.getShootPose().getHeading())
//                )
//                .build();
//
//        return pathToShoot;
//    }

}







