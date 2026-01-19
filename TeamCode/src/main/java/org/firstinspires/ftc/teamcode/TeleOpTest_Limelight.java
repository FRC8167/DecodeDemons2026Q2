package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLFieldMap;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.SubSystems.LimeLightVision;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter;
import org.firstinspires.ftc.teamcode.SubSystems.Vision;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


@Configurable
@TeleOp(name="TestOp LimeLight", group="TestOps")
public class TeleOpTest_Limelight extends OpMode {

    // static TelemetryManager tmPanels;

    LimeLightVision limey;

    @Override
    public void init() {
        limey = new LimeLightVision(hardwareMap.get(Limelight3A.class, "limelight"), 1, true);
//        tmPanels = PanelsTelemetry.INSTANCE.getTelemetry();
    }


    @Override
    public void loop() {

        limey.periodic();

        if (limey.getAprilTags() != null) {
            for (LLResultTypes.FiducialResult aTag : limey.getAprilTags()) {
                telemetry.addData("Tag Id", aTag.getFiducialId()); // The ID number of the fiducial

                telemetry.addData("Field Spacea (X,Y,Z)", "(%.0f, %.0f, %.0f)",
                        aTag.getRobotPoseFieldSpace().getPosition().x,
                        aTag.getRobotPoseFieldSpace().getPosition().y,
                        aTag.getRobotPoseFieldSpace().getPosition().z);
                telemetry.addData("Distance Unit", aTag.getRobotPoseFieldSpace().getPosition().unit);

                telemetry.addData("Target Spacea (X,Y,Z)", "(%.0f, %.0f, %.0f)",
                        aTag.getRobotPoseTargetSpace().getPosition().x,
                        aTag.getRobotPoseTargetSpace().getPosition().y,
                        aTag.getRobotPoseTargetSpace().getPosition().z);
                telemetry.addData("Distance Unit", aTag.getRobotPoseTargetSpace().getPosition().unit);
            }
        } else {
            telemetry.addLine("No target tags (20–24) detected.");
        }

        telemetry.update();
    }


}
