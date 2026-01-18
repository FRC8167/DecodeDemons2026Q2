package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter;
import org.firstinspires.ftc.teamcode.SubSystems.Vision;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


@Configurable
@TeleOp(name="TeleOp_Look", group="TestOps")
public class Telop_LookUpTable extends OpMode {

    private final Robot robot = Robot.getInstance();
    static TelemetryManager tmPanels;

    public static double cmd;
    public static double increment = 25;

    boolean xPressed, yPressed;

    Vision vision;
    Shooter shooter;

    @Override
    public void init() {
        MotorEx shooterMotor = new MotorEx(hardwareMap, "Shooter").setCachingTolerance(0.01);
        shooter  = new Shooter(shooterMotor);
        try {
            vision   = new Vision(hardwareMap.get(WebcamName.class, "Webcam1"));
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        tmPanels = PanelsTelemetry.INSTANCE.getTelemetry();
        xPressed = false;
        yPressed = false;
        cmd = 3000;
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
