package org.firstinspires.ftc.teamcode.SubSystems;

import android.util.Size;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraIntrinsics;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

public class Vision extends SubsystemBase {

    /* --- Target Tag IDs we care about --- */
    private final int[] TARGET_TAGS = {20, 21, 22, 23, 24};

    /* Camera settings */
    private long CAMERA_EXPOSURE = 6;    // milliseconds
    private int CAMERA_GAIN = 250;

    /* Camera and processors */
    private WebcamName atCamera = null;
    private AprilTagProcessor aprilTag = null;
    private VisionPortal visionPortal = null;

    public ElapsedTime runtime = new ElapsedTime();

    private List<AprilTagDetection> currentDetections = new ArrayList<>();

    /* ===================== CONSTRUCTOR ===================== */
    public Vision(WebcamName camera) throws InterruptedException {
        atCamera = camera;
        buildAprilTagProcessor();
        buildVisionPortal();
    }

    /* ===================== BUILDERS ===================== */
    private void buildAprilTagProcessor() {
        aprilTag = new AprilTagProcessor.Builder()
                //.setLensIntrinsics(intrinsics.fx, intrinsics.fy, intrinsics.cx, intrinsics.cy)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();
        aprilTag.setDecimation(2);
    }


    private void buildVisionPortal() throws InterruptedException {
        visionPortal = new VisionPortal.Builder()
                .setCamera(atCamera)
                .addProcessors(aprilTag)
                .setCameraResolution(new Size(640, 480))
                .build();

        // Wait until camera is streaming
        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            Thread.yield();
        }

        setManualExposure(CAMERA_EXPOSURE, CAMERA_GAIN);

        // Start with AprilTags disabled to save CPU
        enableAprilTagDetection();
    }

    /* ===================== CAMERA CONTROLS ===================== */
    public void setManualExposure(long exposureMS, int cameraGain) {
        if (visionPortal == null) return;

        ExposureControl exposure = visionPortal.getCameraControl(ExposureControl.class);
        if (exposure.isExposureSupported()) {
            exposure.setMode(ExposureControl.Mode.Manual);
            exposure.setExposure(exposureMS, TimeUnit.MILLISECONDS);

            GainControl gain = visionPortal.getCameraControl(GainControl.class);
            gain.setGain(cameraGain);
        }
    }

    public void enableAprilTagDetection() {
        if(visionPortal != null) {
            visionPortal.setProcessorEnabled(aprilTag, true);
        }
    }

    public void disableAprilTagDetection() {
        if (visionPortal != null){
            visionPortal.setProcessorEnabled(aprilTag, false);
        }
    }

    /* ===================== APRILTAG FUNCTIONS ===================== */
    /** Refresh detection list */
    public void scanForAprilTags() {
        if (aprilTag != null) {
            currentDetections = aprilTag.getDetections();
        }
    }

    //Find ALL tags
    public List<AprilTagDetection> getAllDetections() {
        return currentDetections;
    }

    //Return only the possible obelisk tags
    public List<AprilTagDetection> getTargetTags() {
        List<AprilTagDetection> filtered = new ArrayList<>();
        for (AprilTagDetection detection : currentDetections) {
            for (int id : TARGET_TAGS) {
                if (detection.id == id) {
                    filtered.add(detection);
                }
            }
        }
        return filtered;
    }

    //Only TRUE if tags 20, 21, 22, 23, or 24
    public boolean targetTagVisible() {
        return !getTargetTags().isEmpty();
    }

    /** First tag found from 20–24 (or null) */
    public AprilTagDetection getFirstTargetTag() {
        List<AprilTagDetection> filtered = getTargetTags();
        return filtered.isEmpty() ? null : filtered.get(0);
    }

    //Make a string from the tag ID representing the motif pattern
    public String getTagCode(AprilTagDetection tag) {
        if (tag == null) return "None";
        switch (tag.id) {
            case 20: return "BLUE GOAL";
            case 21: return "GPP";
            case 22: return "PGP";
            case 23: return "PPG";
            case 24: return "RED GOAL";
            default: return "Unknown";
        }
    }

//    public String[] getMotifPattern() {
//        AprilTagDetection tag = getFirstTargetTag();
//        if (tag == null) return null;
//
//        switch (tag.id) {
//            case 21: return new String[]{"GREEN", "PURPLE", "PURPLE"}; // GPP
//            case 22: return new String[]{"PURPLE", "GREEN", "PURPLE"}; // PGP
//            case 23: return new String[]{"PURPLE", "PURPLE", "GREEN"}; // PPG
//            default: return null; // Goals don't define motif
//        }
//    }

    public ColorMatch.ArtifactColor[] getMotifPattern() {
        AprilTagDetection tag = getFirstTargetTag();
        if (tag == null) return null;

        switch (tag.id) {
            case 21:
                return new ColorMatch.ArtifactColor[]{
                        ColorMatch.ArtifactColor.GREEN,
                        ColorMatch.ArtifactColor.PURPLE,
                        ColorMatch.ArtifactColor.PURPLE
                };
            case 22:
                return new ColorMatch.ArtifactColor[]{
                        ColorMatch.ArtifactColor.PURPLE,
                        ColorMatch.ArtifactColor.GREEN,
                        ColorMatch.ArtifactColor.PURPLE
                };
            case 23:
                return new ColorMatch.ArtifactColor[]{
                        ColorMatch.ArtifactColor.PURPLE,
                        ColorMatch.ArtifactColor.PURPLE,
                        ColorMatch.ArtifactColor.GREEN
                };
            default:
                return null;
        }
    }



    public double getDistanceToGoal() {
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == 20  || detection.id == 24) {
                return (detection.ftcPose.range);
            }
        }
        return Double.NaN; //what is Dave?
    }


    @Override
    public void periodic() {
        scanForAprilTags();
    }

}
