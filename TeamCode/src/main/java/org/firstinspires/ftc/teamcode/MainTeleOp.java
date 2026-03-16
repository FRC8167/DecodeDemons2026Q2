package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Commands.CancelPedroCommand;
import org.firstinspires.ftc.teamcode.Commands.DetectArtifactCommand;
import org.firstinspires.ftc.teamcode.Commands.DriveToPoseCommand;
import org.firstinspires.ftc.teamcode.Commands.HoldPoseCommand;
import org.firstinspires.ftc.teamcode.Commands.KickCommand;
import org.firstinspires.ftc.teamcode.Commands.NestCommand;
import org.firstinspires.ftc.teamcode.Commands.RotateOneSlotCommand;
import org.firstinspires.ftc.teamcode.Commands.ShootCaseCommand;
import org.firstinspires.ftc.teamcode.Commands.ShooterSmartSpinUpCommand;
import org.firstinspires.ftc.teamcode.Commands.ShooterSpinUpCommand;
import org.firstinspires.ftc.teamcode.Commands.VisionCommand;
import org.firstinspires.ftc.teamcode.SubSystems.ColorMatch;
import org.firstinspires.ftc.teamcode.SubSystems.JugglerAbsolute_EXP;

@Configurable
//@Disabled
@TeleOp(name="MainTeleOp", group="Competition")
public class MainTeleOp extends CommandOpMode {

    public GamepadEx driver;
    public GamepadEx operator;
//    public ElapsedTime loopTimer;
    private final Robot robot = Robot.getInstance();
//    static TelemetryManager telemetryM;
    Pose currentPose;

    public static double current_velocity = 3200;
    public double increment = 25;

    private boolean automatedDrive = false;

    // For Loop Timer
    long startTime, endTime, loopTime;


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

        telemetry.addData("InitCount: ", Robot.initCount);
        telemetry.update();

//        robot.mecanumDrive.setDefaultCommand(new DriveCommand(robot.mecanumDrive, gamepad1));

        // Comment for increasing loop time experiment
//        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        //end pose held in robot
        //if auto ran, last pose is used or else default of 24,24,0
        Pose startPose = robot.autoEndPose != null ? robot.autoEndPose : new Pose(24, 24, 0);
        robot.follower.setStartingPose(startPose);
        robot.follower.update();
        schedule(new VisionCommand(robot.vision));
        schedule(new DetectArtifactCommand(robot.rgbLight, robot.colorMatch, gamepad2)); //, robot.shooter));

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
//                whenPressed(new RotateXSlotsCommand(robot.juggler, Juggler.Direction.CW, 1));
        whenPressed(new RotateOneSlotCommand(robot.juggler, JugglerAbsolute_EXP.Direction.CW));

        operator.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).
//                whenPressed(new RotateXSlotsCommand(robot.juggler, Juggler.Direction.CCW, 1));
                whenPressed(new RotateOneSlotCommand(robot.juggler, JugglerAbsolute_EXP.Direction.CCW));

        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(
                        new SequentialCommandGroup(
                                new KickCommand(robot.slide),
                                new NestCommand(robot.slide)
                        )
                );

//        operator.getGamepadButton(GamepadKeys.Button.START)
//                .whenPressed(new InstantCommand(() -> robot.popper.midServo()));


        operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(robot.vision::latchMotif));

//        operator.getGamepadButton(GamepadKeys.Button.DPAD_UP)
//                .whenPressed(
//                        new InstantCommand(()->robot.juggler.goHome())
//                );

        operator.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new ShootCaseCommand(robot.juggler, robot.slide, robot.shooter, robot.colorMatch, robot.vision)
                );

        operator.getGamepadButton(GamepadKeys.Button.X)
                .whileHeld(new RunCommand(() -> robot.juggler.startSlowSpin(JugglerAbsolute_EXP.Direction.CW), robot.juggler))
                .whenReleased(new InstantCommand(() -> robot.juggler.snapToNearestSlot(), robot.juggler));

//        operator.getGamepadButton(GamepadKeys.Button.BACK)
//                        .whenPressed(new InstantCommand(()-> robot.juggler.confirmHome()));

        /* ****************************** DRIVER CONTROLS ****************************** */

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> automatedDrive = true),
                                new HoldPoseCommand(driver),
                                new InstantCommand(() -> automatedDrive = false)
                        )
                );


        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(robot.mecanumDrive::enableSnailDrive))
                .whenReleased(new InstantCommand(robot.mecanumDrive::disableSnailDrive));

        //must be HELD DOWN to hold the position
        //operator can still use anything
        //once driver releases and moves joystick, the hold ends

        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new KickCommand(robot.slide));

        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new NestCommand(robot.slide));

//        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
//                .whenPressed(new InstantCommand(()->robot.juggler.jogThree(Juggler.Direction.CCW)));
//
//        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
//                .whenPressed(new InstantCommand(()->robot.juggler.jogThree(Juggler.Direction.CCW)));


//        driver.getGamepadButton(GamepadKeys.Button.BACK)
//                .whenPressed(new CancelPedroCommand());

        driver.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> automatedDrive = true),
                                new DriveToPoseCommand(robot.getShootPose(), driver),
                                new CancelPedroCommand(),
                                new InstantCommand(() -> automatedDrive = false)
                        )
                );

        //driver-assisted shoot commands
        driver.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> automatedDrive = true),
                                new DriveToPoseCommand(robot.getShootPose(), driver),
                                new ShootCaseCommand(robot.juggler, robot.slide, robot.shooter, robot.colorMatch, robot.vision),
                                new InstantCommand(()->robot.shooter.stop()),
                                new CancelPedroCommand(),
                                new InstantCommand(() -> automatedDrive = false)
                                        //End the path hold
//                                        new InstantCommand(() -> robot.follower.breakFollowing())

                        )
                );



        driver.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> automatedDrive = true),
                                new DriveToPoseCommand(robot.getParkPose(), driver),
                                new CancelPedroCommand(),
                                new InstantCommand(() -> automatedDrive = false)
                        )
                );

        // Not Competition Ready - Enable when Limelight is added
//        driver.getGamepadButton(GamepadKeys.Button.Y)
//                .whenPressed( new InstantCommand(()-> robot.mecanumDrive.setDriveMode(MecanumDrive.DriveMode.CONSTANT_HEADING)))
//                .whileHeld(   new InstantCommand(()-> robot.mecanumDrive.setBearings(robot.limey.getYawToTag(), 0)))
//                .whenReleased(new InstantCommand(()-> robot.mecanumDrive.setDriveMode(MecanumDrive.DriveMode.NORMAL))
//                );

        startTime = System.currentTimeMillis();

    }

    @Override
    public void run() {

        super.run();

        if (!automatedDrive) {
//            robot.follower.setTeleOpDrive(
//                    -gamepad1.left_stick_y,
//                    gamepad1.left_stick_x,
//                    gamepad1.right_stick_x
//            );
            robot.follower.breakFollowing();
            robot.mecanumDrive.drive(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x
            );
        }

        robot.follower.update();
        robot.autoEndPose = robot.follower.getPose();




//        if (tag != null) {
//            telemetry.addLine("Target Tag Detected!");
//            telemetry.addData("ID", tag.getFiducialId());
//            Position targetInCamera = tag.getTargetPoseCameraSpace().getPosition();
//            telemetry.addData("Center", "(%.0f, %.0f)", targetInCamera.x, targetInCamera.z); //Note: These values may be swapped
//            telemetry.addData("Range (in)", "%.1f", Math.sqrt(Math.pow(targetInCamera.x, 2) + Math.pow(targetInCamera.y, 2) + Math.pow(targetInCamera.z, 2)));
//        } else {
//            telemetry.addLine("No target tags (20–24) detected.");
//        }

        //Trying to add limelight replacement for above
//        LLResult result = robot.vision.getResult();
//
//        if (result == null) {
//            telemetry.addLine("AprilTags: No valid Limelight result");
//        } else {
//            List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
//
//            if (tags == null || tags.isEmpty()) {
//                telemetry.addLine("AprilTags: None detected");
//            } else {
//                for (LLResultTypes.FiducialResult tag : tags) {
//                    telemetry.addData(
//                            LimeLightVision.tagIdLookup(tag.getFiducialId()),
//                            tag.getFiducialId()
//                    );
//                }
//            }
//        }

//        telemetry.addData("jugggler count", robot.juggler.getCurrentPosition());
//        telemetry.addData("jugggler target", robot.juggler.getTargetPosition());
//        telemetry.addData("autoEndPose", robot.autoEndPose.toString());
//        telemetry.addData("FollowerX", Math.round(robot.follower.getPose().getX() * 100) / 100.0);
//        telemetry.addData("FollowerY", Math.round(robot.follower.getPose().getY() * 100) / 100.0);
//        telemetry.addData("FollowerH", Math.round(Math.toDegrees(robot.follower.getPose().getHeading()) * 100) / 100.0);
        telemetry.addData("Distance to Goal", robot.vision.getGoalDistance());


        ColorMatch.ArtifactColor[] motif = robot.vision.getLatchedMotif();


        // Moved from telemetryM to telemetry
        if (motif != null) {
            telemetry.addData("Obelisk Motif",motif[0] + " - " + motif[1] + " - " + motif[2]
            );
        } else {
            telemetry.addData("Obelisk Motif", "Not latched");
        }

//        telemetry.addData("Shooter Velocity (RPM)", robot.shooter.getRPM());
//        telemetry.addData("Shooter Ready?", robot.shooter.atTargetVelocity());
//        telemetry.addData("Juggler counts", robot.juggler.getCurrentPosition());

        telemetry.addData("Slot 0", robot.colorMatch.detectColor(ColorMatch.Slot.SLOT_0));
//        telemetry.addData("dist",robot.colorMatch.getDistance(ColorMatch.Slot.SLOT_0));
//        telemetry.addData("h",robot.colorMatch.getHSV(ColorMatch.Slot.SLOT_0)[0]);

        telemetry.addData("Slot 1", robot.colorMatch.detectColor(ColorMatch.Slot.SLOT_1));
//        telemetry.addData("dist",robot.colorMatch.getDistance(ColorMatch.Slot.SLOT_1));
//        telemetry.addData("h",robot.colorMatch.getHSV(ColorMatch.Slot.SLOT_1)[0]);

        telemetry.addData("Slot 2", robot.colorMatch.detectColor(ColorMatch.Slot.SLOT_2));
//        telemetry.addData("dist",robot.colorMatch.getDistance(ColorMatch.Slot.SLOT_2));
//        telemetry.addData("h",robot.colorMatch.getHSV(ColorMatch.Slot.SLOT_2)[0]);

//        telemetryM.addData("Shooter Ready?", robot.shooter.atTargetVelocity());
//        telemetryM.addData("Current Velocity", current_velocity);

        endTime   = System.currentTimeMillis();
        loopTime  = endTime - startTime;
        startTime = endTime;
        telemetry.addData("Loop Time [ms]", loopTime);
        telemetry.addLine();
        telemetry.addData("AutoDrive: ", automatedDrive);

        telemetry.update();
//        telemetryM.update(telemetry);

    }


    @Override
    public void end() {
        robot.autoEndPose = robot.follower.getPose();
    }

    public Pose getAutoEndPose() {
        return robot.autoEndPose;
    }




}












//package org.firstinspires.ftc.teamcode;
//
//import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.TelemetryManager;
//import com.pedropathing.geometry.Pose;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.seattlesolvers.solverslib.command.CommandOpMode;
//import com.seattlesolvers.solverslib.command.InstantCommand;
//import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
//import com.seattlesolvers.solverslib.command.RunCommand;
//import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
//import com.seattlesolvers.solverslib.command.WaitCommand;
//import com.seattlesolvers.solverslib.gamepad.GamepadEx;
//import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
//
//import org.firstinspires.ftc.teamcode.Commands.CancelPedroCommand;
//import org.firstinspires.ftc.teamcode.Commands.DetectArtifactCommand;
//import org.firstinspires.ftc.teamcode.Commands.DriveCommand;
//import org.firstinspires.ftc.teamcode.Commands.DriveToPoseCommand;
//import org.firstinspires.ftc.teamcode.Commands.HoldPoseCommand;
//import org.firstinspires.ftc.teamcode.Commands.RotateOneSlotCommand;
//import org.firstinspires.ftc.teamcode.Commands.ShootCaseCommand;
//import org.firstinspires.ftc.teamcode.Commands.ShooterSmartSpinUpCommand;
//import org.firstinspires.ftc.teamcode.Commands.ShooterSpinUpCommand;
//import org.firstinspires.ftc.teamcode.Commands.VisionCommand;
//import org.firstinspires.ftc.teamcode.SubSystems.ColorMatch;
//import org.firstinspires.ftc.teamcode.SubSystems.Juggler;
//import org.firstinspires.ftc.teamcode.SubSystems.MecanumDrive;
//import org.firstinspires.ftc.teamcode.SubSystems.Popper;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//
//@Configurable
////@Disabled
//@TeleOp(name="MainTeleOp", group="Competition")
//public class MainTeleOp extends CommandOpMode {
//
//    public GamepadEx driver;
//    public GamepadEx operator;
//    public ElapsedTime timer;
//    private final Robot robot = Robot.getInstance();
//    static TelemetryManager telemetryM;
//    Pose currentPose;
//
//    public static double current_velocity = 3200;
//    public double increment = 25;
//
//
//    private Pose startPose;
////    private Pose autoEndPose;
//
//
//    @Override
//    public void initialize() {
//
//        // Must have for all opModes
//        Robot.OP_MODE_TYPE = Robot.OpModeType.TELEOP;
//        // Resets the command scheduler
//        super.reset();
//
//        //Initialize the robot
//        try {
//            robot.init(hardwareMap);
//        } catch (InterruptedException e) {
//            throw new RuntimeException(e);
//        }
//        if (robot.autoEndPose == null) {
//            startPose = new Pose(0, 0, 0);
//        } else {
//            startPose = robot.autoEndPose;
//        }
//
//        robot.mecanumDrive.setDefaultCommand(new DriveCommand(robot.mecanumDrive, gamepad1));
//        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
//
//        //end pose held in robot
//        //if auto ran, last pose is used or else default of 24,24,0
//        Pose startPose = robot.autoEndPose != null ? robot.autoEndPose : new Pose(24, 24, 0);
//        robot.follower.setStartingPose(startPose);
//        robot.follower.update();
//        schedule(new VisionCommand(robot.vision));
//        schedule(new DetectArtifactCommand(robot.rgbLight, robot.colorMatch, gamepad2)); //, robot.shooter));
//
//        driver = new GamepadEx(gamepad1);
//        operator = new GamepadEx(gamepad2);
//
//
//        //******OPERATOR CONTROLS*****
//        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
//                .whileHeld(new ShooterSmartSpinUpCommand(robot.shooter, robot.vision, robot.colorMatch))
//                .whenReleased(new ShooterSpinUpCommand(robot.shooter, 0.0));
//
//
////        operator.getGamepadButton(GamepadKeys.Button.BACK)
////                .whileHeld(new ShooterSpinUpCommand(robot.shooter, -1000.0))
////                .whenReleased(new ShooterSpinUpCommand(robot.shooter, 0.0));
//
//
//        operator.getGamepadButton(GamepadKeys.Button.A)
//                .whenPressed(new InstantCommand(robot.intake::forward));
//
//        operator.getGamepadButton(GamepadKeys.Button.B)
//                .whenPressed(new InstantCommand(robot.intake::reverse));
//
//        operator.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).
//                whenPressed(new RotateOneSlotCommand(robot.juggler, Juggler.Direction.CW));
//
//        operator.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).
//                whenPressed(new RotateOneSlotCommand(robot.juggler, Juggler.Direction.CCW));
//
//        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
//                .whenPressed(
//                        new SequentialCommandGroup(
//                                new InstantCommand(()->robot.popper.set(Popper.PopperState.KICK)),
//                                new WaitCommand(500),
//                                new InstantCommand(()->robot.popper.set(Popper.PopperState.RESET))
//                        )
//                );
//
//        operator.getGamepadButton(GamepadKeys.Button.START)
//                .whenPressed(new InstantCommand(() -> robot.popper.midServo()));
//
//
//        operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
//                .whenPressed(new InstantCommand(robot.vision::latchMotif));
//
//
//
//        operator.getGamepadButton(GamepadKeys.Button.Y)
//                .whenPressed(new ShootCaseCommand(robot.juggler, robot.popper, robot.shooter, robot.colorMatch, robot.vision)
//                );
//
//        operator.getGamepadButton(GamepadKeys.Button.X)
//                .whileHeld(new RunCommand(() -> robot.juggler.startSlowSpin(Juggler.Direction.CW), robot.juggler))
//                .whenReleased(new InstantCommand(() -> robot.juggler.Snap(), robot.juggler));
//
//
//
//        /* ****************************** DRIVER CONTROLS ****************************** */
//
//        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
//                .whileHeld(new HoldPoseCommand(robot.follower.getPose(), driver));
//
//
//        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
//                .whenPressed(new InstantCommand(robot.mecanumDrive::enableSnailDrive))
//                .whenReleased(new InstantCommand(robot.mecanumDrive::disableSnailDrive));
//
//        //must be HELD DOWN to hold the position
//        //operator can still use anything
//        //once driver releases and moves joystick, the hold ends
//
//        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP)
//                .whenPressed(new InstantCommand(()->robot.popper.set(Popper.PopperState.KICK)));
//
//        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
//                .whenPressed(new InstantCommand(()->robot.popper.set(Popper.PopperState.RESET)));
//
//
//        driver.getGamepadButton(GamepadKeys.Button.BACK)
//                .whenPressed(new CancelPedroCommand());
//
//        driver.getGamepadButton(GamepadKeys.Button.A)
//                .whenPressed(
//                        new SequentialCommandGroup(
//                                new DriveToPoseCommand(robot.getShootPose(), driver),
//                                //Hold the pose as defensive strategy
//                                new HoldPoseCommand(robot.getShootPose(), driver)
//                        )
//
//                );
//
//        //driver-assisted shoot commands
//        driver.getGamepadButton(GamepadKeys.Button.B)
//                .whenPressed(
//                        new SequentialCommandGroup(
//                                new DriveToPoseCommand(robot.getShootPose(), driver),
//                                new ParallelCommandGroup(
//                                    //Hold the pose as defensive strategy
//                                    new HoldPoseCommand(robot.getShootPose(), driver),
//                                    //Motif shoot command
//                                    new ShootCaseCommand(robot.juggler, robot.popper, robot.shooter, robot.colorMatch, robot.vision)
//                                ),
//                                new ParallelCommandGroup(
//                                    //Spin down shooter
//                                    new ShooterSpinUpCommand(robot.shooter, 0),
//                                    //End the path hold
//                                    new InstantCommand(() -> robot.follower.breakFollowing())
//                                )
//                        )
//                );
//
////        driver.getGamepadButton(GamepadKeys.Button.X).whenPressed(
////            new InstantCommand(()-> robot.shooter.setVelocity(current_velocity)));
//
//        driver.getGamepadButton(GamepadKeys.Button.Y)
//                .whenPressed( new InstantCommand(()-> robot.mecanumDrive.setDriveMode(MecanumDrive.DriveMode.CONSTANT_HEADING)))
//                .whileHeld(   new InstantCommand(()-> robot.mecanumDrive.setBearings(robot.follower.getPose().getHeading(), robot.vision.getTargetBearing())))
//                .whenReleased(new InstantCommand(()-> robot.mecanumDrive.setDriveMode(MecanumDrive.DriveMode.NORMAL))
//                );
//
//
//
//    }
//
//    @Override
//    public void run() {
//        super.run();
//        robot.follower.update();
//        robot.autoEndPose = robot.follower.getPose();
//        AprilTagDetection tag = robot.vision.getFirstTargetTag();
//
//
//        if (tag != null) {
//            telemetry.addLine("Target Tag Detected!");
//            telemetry.addData("ID", tag.id);
//            telemetry.addData("Center", "(%.0f, %.0f)", tag.center.x, tag.center.y);
//            telemetry.addData("Range (in)", "%.1f", tag.ftcPose.range);
//        } else {
//            telemetry.addLine("No target tags (20–24) detected.");
//        }
//
//
//        telemetry.addData("autoEndPose", robot.autoEndPose.toString());
//        telemetry.addData("FollowerX", Math.round(robot.follower.getPose().getX() * 100) / 100.0);
//        telemetry.addData("FollowerY", Math.round(robot.follower.getPose().getY() * 100) / 100.0);
//        telemetry.addData("FollowerH", Math.round(Math.toDegrees(robot.follower.getPose().getHeading()) * 100) / 100.0);
//        telemetry.addData("Distance to Goal", robot.vision.getDistanceToGoal());
//
//        ColorMatch.ArtifactColor[] motif = robot.vision.getLatchedMotif();
//
//        if (motif != null) {
//            telemetryM.addData(
//                    "Obelisk Motif",
//                    motif[0] + " - " + motif[1] + " - " + motif[2]
//            );
//        } else {
//            telemetryM.addData("Obelisk Motif", "Not latched");
//        }
////        telemetryM.addData("Obelisk Motif", robot.vision.getLatchedMotifString());
//
//
//        telemetry.addData("Popperstate", robot.popper.getPopperState());
//        telemetry.addData("Shooter Velocity (RPM)", robot.shooter.getRPM());
//        telemetry.addData("Shooter Ready?", robot.shooter.atTargetVelocity());
////        telemetry.addData("Juggler counts", robot.juggler.getCurrentPosition());
//
//
//
//        telemetryM.addData("Slot 0", robot.colorMatch.detectColor(ColorMatch.Slot.SLOT_0));
//        telemetryM.addData("Slot 1", robot.colorMatch.detectColor(ColorMatch.Slot.SLOT_1));
//        telemetryM.addData("Slot 2", robot.colorMatch.detectColor(ColorMatch.Slot.SLOT_2));
//        telemetryM.addData("Shooter Ready?", robot.shooter.atTargetVelocity());
//        telemetryM.addData("Current Velocity", current_velocity);
//        telemetryM.update(telemetry);
//
//    }
//
//
//    @Override
//    public void end() {
//        robot.autoEndPose = robot.follower.getPose();
//    }
//
//    public Pose getAutoEndPose() {
//        return robot.autoEndPose;
//    }
//
//
//
//
//}
//
//
//
//
//
//
//
