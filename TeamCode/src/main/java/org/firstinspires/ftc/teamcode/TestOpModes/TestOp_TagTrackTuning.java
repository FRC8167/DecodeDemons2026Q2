package org.firstinspires.ftc.teamcode.TestOpModes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.Commands.DriveCommand;
import org.firstinspires.ftc.teamcode.Commands.HoldPoseCommand;
import org.firstinspires.ftc.teamcode.Commands.VisionCommand;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.SubSystems.MecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Disabled
@Configurable
//@Disabled
@TeleOp(name="Tag Track Tuning", group="TestOps")
public class TestOp_TagTrackTuning extends CommandOpMode {

    public GamepadEx driver;
    private final Robot robot = Robot.getInstance();
    static TelemetryManager telemetryM;

    public static double Kp = 0.1, FF = 0;

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

        robot.mecanumDrive.setDefaultCommand(new DriveCommand(robot.mecanumDrive, gamepad1));
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        //end pose held in robot
        //if auto ran, last pose is used or else default of 24,24,0
        Pose startPose = robot.autoEndPose != null ? robot.autoEndPose : new Pose(24, 24, 90);
        robot.follower.setStartingPose(startPose);
        robot.follower.update();
        schedule(new VisionCommand(robot.vision));

        driver = new GamepadEx(gamepad1);

        /* ****************************** DRIVER CONTROLS ****************************** */

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(new HoldPoseCommand(robot.follower.getPose(), driver));


        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(robot.mecanumDrive::enableSnailDrive))
                .whenReleased(new InstantCommand(robot.mecanumDrive::disableSnailDrive));


    //    driver.getGamepadButton(GamepadKeys.Button.Y)
    //            .whenPressed( new InstantCommand(()-> robot.mecanumDrive.setDriveMode(MecanumDrive.DriveMode.CONSTANT_HEADING)))
    //            .whileHeld(   new InstantCommand(()-> robot.mecanumDrive.setBearings(robot.follower.getPose().getHeading(), robot.vision.getTargetBearing())))
    //            .whenReleased(new InstantCommand(()-> robot.mecanumDrive.setDriveMode(MecanumDrive.DriveMode.NORMAL)));
        driver.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed( new InstantCommand(()-> robot.mecanumDrive.setDriveMode(MecanumDrive.DriveMode.CONSTANT_HEADING)))
                .whenReleased(new InstantCommand(()-> robot.mecanumDrive.setDriveMode(MecanumDrive.DriveMode.NORMAL)));

    }


    @Override
    public void run() {
        super.run();
        robot.follower.update();
        robot.autoEndPose = robot.follower.getPose();

        robot.mecanumDrive.setKpHeading(Kp);
        robot.mecanumDrive.setFFheading(FF);

        //AprilTagDetection tag = robot.vision.getFirstTargetTag();
//        LLResultTypes.FiducialResult tag = robot.vision.getAprilTags().get(0);

//        if (tag != null) {
//            telemetry.addLine("Target Tag Detected!");
//            telemetry.addData("ID", tag.getFiducialId());
//            Position targetInCamera = tag.getTargetPoseCameraSpace().getPosition();
//            telemetry.addData("Center", "(%.0f, %.0f)", targetInCamera.x, targetInCamera.z); //Note: These values may be swapped
//        } else {
//            telemetryM.addLine("No target tags (20–24) detected.");
//        }

        telemetryM.addData("Follower Heading[deg]", Math.round(Math.toDegrees(robot.follower.getPose().getHeading()) * 100) / 100.0);
//        telemetryM.addData("Pinpoint Heading[deg]", Math.round(Math.toDegrees(robot.pinpoint.getHeading(AngleUnit.DEGREES)) * 100) / 100.0);
        telemetryM.addData("Distance to Goal[in]", robot.vision.getGoalDistance());

        telemetryM.update(telemetry);

    }


    @Override
    public void end() {
        robot.autoEndPose = robot.follower.getPose();
    }

    public Pose getAutoEndPose() {
        return robot.autoEndPose;
    }




}








