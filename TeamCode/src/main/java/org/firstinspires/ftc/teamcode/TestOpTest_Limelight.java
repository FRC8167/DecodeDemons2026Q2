package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.LimeLightVision;


@Configurable
@TeleOp(name="TestOp LimeLight", group="TestOps")
public class TestOpTest_Limelight extends OpMode {

     static TelemetryManager tmPanels;

    LimeLightVision limey;

    @Override
    public void init() {
        limey = new LimeLightVision(hardwareMap.get(Limelight3A.class, "LimeLight"), 1, true);
        tmPanels = PanelsTelemetry.INSTANCE.getTelemetry();

    }


    @Override
    public void loop() {

        limey.periodic();

        if (limey.getAprilTags() != null) {
            for (LLResultTypes.FiducialResult aTag : limey.getAprilTags()) {
                tmPanels.addData("Tag Id", aTag.getFiducialId()); // The ID number of the fiducial
                telemetry.addData("Tag Id", aTag.getFiducialId());

                tmPanels.addData("Area", aTag.getTargetArea());
                tmPanels.addData("X degrees", aTag.getTargetXDegreesNoCrosshair());
                tmPanels.addData("Y degrees", aTag.getTargetYDegreesNoCrosshair());

                tmPanels.addData("Pose in Field Space [in]", fieldSpacePos(aTag));
                telemetry.addData("Field Spacea (X,Y,Z) [in]", "(%.00f, %.00f, %.00f)",
                        aTag.getRobotPoseFieldSpace().getPosition().x * 39.37,
                        aTag.getRobotPoseFieldSpace().getPosition().y * 39.37,
                        aTag.getRobotPoseFieldSpace().getPosition().z * 39.37);

//                tmPanels.addData("Distance Unit", aTag.getRobotPoseFieldSpace().getPosition().unit);

                // Tag position as seen from the center of the robot - tag position relative to robot, x=fwd, y=lateral, z=vertical
                tmPanels.addData("Robot Pose in Target Space [in]", robotSpacePos(aTag));
                telemetry.addData("Target Space (X,Y,Z) [in]", "(%.00f, %.00f, %.00f)",
                        aTag.getRobotPoseTargetSpace().getPosition().x * 39.37,
                        aTag.getRobotPoseTargetSpace().getPosition().y * 39.37,
                        aTag.getRobotPoseTargetSpace().getPosition().z * 39.37);
//                tmPanels.addData("Distance Unit", aTag.getRobotPoseTargetSpace().getPosition().unit);

                tmPanels.addData("Distance [in]", limey.getDistanceToGoal()*39.27);
                tmPanels.addData("Yaw Angle to Target", limey.getYawToTarget());
            }
        } else {
            tmPanels.addLine("No target tags (20–24) detected.");
        }

        tmPanels.update();
        telemetry.update();
    }


    @SuppressLint("DefaultLocale")
    private String str(double num) {
        return String.format("(%.0f", num);
    }

    @SuppressLint("DefaultLocale")
    private String fieldSpacePos(LLResultTypes.FiducialResult tag) {
        return String.format("%.00f, %.00f, %.00f",
                tag.getRobotPoseFieldSpace().getPosition().x * 39.37,
                tag.getRobotPoseFieldSpace().getPosition().y * 39.37,
                tag.getRobotPoseFieldSpace().getPosition().z * 39.37);
    }


    @SuppressLint("DefaultLocale")
    private String robotSpacePos(LLResultTypes.FiducialResult tag) {
        return String.format("%.00f, %.00f, %.00f",
                tag.getRobotPoseTargetSpace().getPosition().x * 39.37,
                tag.getRobotPoseTargetSpace().getPosition().y * 39.37,
                tag.getRobotPoseTargetSpace().getPosition().z * 39.37);
    }

}
