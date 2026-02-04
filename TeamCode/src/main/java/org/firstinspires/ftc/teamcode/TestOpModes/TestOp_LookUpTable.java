package org.firstinspires.ftc.teamcode.TestOpModes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.SubSystems.Popper;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter;
import org.firstinspires.ftc.teamcode.SubSystems.Vision;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


@Configurable
@TeleOp(name="Shooter LookUp Table Calibration", group="TestOps")
public class TestOp_LookUpTable extends OpMode {

    private final Robot robot = Robot.getInstance();
    static TelemetryManager tmPanels;

    public static double cmd;
    public static double increment = 25;

    boolean xPressed, yPressed, right_bumperPressed;

    Vision vision;
    Shooter shooter;
    Popper popper;

    @Override
    public void init() {
        MotorEx shooterMotor = new MotorEx(hardwareMap, "Shooter").setCachingTolerance(0.01);
        shooter  = new Shooter(shooterMotor);
        ServoEx popperServoL = new ServoEx(hardwareMap, "popperServoL");
        ServoEx popperServoR = new ServoEx(hardwareMap, "popperServoR");
        popper = new Popper(popperServoL, popperServoR);
        try {
            vision   = new Vision(hardwareMap.get(WebcamName.class, "Webcam1"));
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        tmPanels = PanelsTelemetry.INSTANCE.getTelemetry();
        xPressed = false;
        yPressed = false;
        right_bumperPressed = false;
        cmd = 3000;
        popper.set(Popper.PopperState.RESET);
    }


//    @Override
//    public void start() {
//
//    }

    @Override
    public void loop() {

        vision.scanForAprilTags();
        AprilTagDetection tag = vision.getFirstTargetTag();

        if (tag != null) {
            tmPanels.addLine("Target Tag Detected!");
//            telemetry.addData("ID", tag.id);
//            telemetry.addData("Center", "(%.0f, %.0f)", tag.center.x, tag.center.y);
            tmPanels.addData("Range (in)", tag.ftcPose.range);
        } else {
            tmPanels.addLine("No target tags (20–24) detected.");
        }


        if(gamepad1.x & !xPressed) {
            cmd -= increment;
        }

        if(gamepad1.y & !yPressed) {
            cmd += increment;
        }

        if(gamepad1.right_bumper & !right_bumperPressed) {
            if(shooter.atTargetVelocity()) {
                popper.set(Popper.PopperState.KICK);
                new WaitCommand(250);
                popper.set(Popper.PopperState.KICK);
            }
        }

        shooter.setVelocity(cmd);
        shooter.periodic();
        vision.periodic();

        tmPanels.addData("Commanded RPM", cmd);
        tmPanels.addData("Shooter Velocity (RPM)", shooter.getRPM());
        tmPanels.addData("Shooter Ready?", shooter.atTargetVelocity());
        telemetry.addData("Commanded RPM", cmd);
        telemetry.addData("Shooter Velocity (RPM)", shooter.getRPM());
        telemetry.addData("Shooter Ready?", shooter.atTargetVelocity());
        tmPanels.update();
        tmPanels.update();

        xPressed = gamepad1.x;
        yPressed = gamepad1.y;

    }


}
