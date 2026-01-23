package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public class LimeLightVision extends SubsystemBase {

    private final Limelight3A ll;
    private LLResult result;

//    Position defaultPosition = new Position(DistanceUnit.INCH, 0,0,0,0);
//    YawPitchRollAngles defaultAngles = new YawPitchRollAngles(AngleUnit.DEGREES,0,0,0,0);
//    private Pose3D robotPose = new Pose3D(defaultPosition, defaultAngles);

    List<LLResultTypes.FiducialResult> fiducials;


    // From the web cam vision subsystem
    private ColorMatch.ArtifactColor[] latchedMotif = null;


    /**
     * Create a Limelight vision object
     * @param limeLightCamera Camera as defined in the hardware map
     * @param pipeLine Pipeline to initialize the limelight to analyze
     * @param Enable_Immediately Set true to start the image acquisition immediately. Alternately you can call the start() method.
     */
    public LimeLightVision(Limelight3A limeLightCamera, int pipeLine, boolean Enable_Immediately) {
        ll = limeLightCamera;
        ll.setPollRateHz(100);
        ll.pipelineSwitch(pipeLine);
        if(Enable_Immediately) { start(); }
    }


    @Override
    public void periodic() {
        result = ll.getLatestResult();
        
        if (result.isValid() && result != null) {
//            // Access general information


            // Get April Tag results
            fiducials = result.getFiducialResults();
        } else fiducials = null;
//            for (LLResultTypes.FiducialResult fiducial : fiducials) {
//                int id = fiducial.getFiducialId(); // The ID number of the fiducial
//                double x = fiducial.getTargetXDegrees(); // Where it is (left-right)
//                double y = fiducial.getTargetYDegrees(); // Where it is (up-down)
//        }

//            fiducial.getRobotPoseTargetSpace(); // Robot pose relative to the AprilTag Coordinate System (Most Useful)
//            fiducial.getCameraPoseTargetSpace(); // Camera pose relative to the AprilTag (useful)
//            fiducial.getRobotPoseFieldSpace(); // Robot pose in the field coordinate system based on this tag alone (useful)
//            fiducial.getTargetPoseCameraSpace(); // AprilTag pose in the camera's coordinate system (not very useful)
//            fiducial.getTargetPoseRobotSpace(); // AprilTag pose in the robot's coordinate system (not very useful)
//        }
    }


//    private double calcDistanceToTag() {
//        double scale = 1;
//        double pixelArea = tag.getTa();
//        return pixelArea * scale;
//    }


//    public Pose3D getPose() {
//        return robotPose;
//    }


    public void setPipeline(int pipeline) {
        ll.pipelineSwitch(pipeline);
    }


//    public double getDistanceToTag(int TagId) {
//        return distanceToTag;
//    }


    public void start() {
        ll.start();
    }


    public void stop() {
        ll.stop();
    }


    /**
     * Returns the LimeLight status object that contains the following:
     * Camera Name, temperature, CPU usage, frames per second, current pipeline and pipeline type
     * @return Limelight status object
     */
    public LLStatus getStatus() {
        return  ll.getStatus();
    }

    public LLResult getALLResults() {
        if (result != null && result.isValid()) return result;
        else return null;
    }

    public List<LLResultTypes.FiducialResult> getAprilTags() {
        return fiducials;
    }


    /* **************************** From the web cam vision subsystem *************************** */

    public ColorMatch.ArtifactColor[] getMotifPattern() {

        if(fiducials != null && !fiducials.isEmpty()) {
            // Use first tag in View
            switch (fiducials.get(0).getFiducialId()) {
                case 21:  // GPP
                    return new ColorMatch.ArtifactColor[]{ColorMatch.ArtifactColor.GREEN, ColorMatch.ArtifactColor.PURPLE, ColorMatch.ArtifactColor.PURPLE};
                case 22:  // PGP
                    return new ColorMatch.ArtifactColor[]{ColorMatch.ArtifactColor.PURPLE, ColorMatch.ArtifactColor.GREEN, ColorMatch.ArtifactColor.PURPLE};
                case 23:  // PPG
                    return new ColorMatch.ArtifactColor[]{ColorMatch.ArtifactColor.PURPLE, ColorMatch.ArtifactColor.PURPLE, ColorMatch.ArtifactColor.GREEN};
                default:
                    return null;
            }
        } else return null;
    }


    public double getDistanceToGoal() {
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            if(fiducial.getFiducialId() == 20 || fiducial.getFiducialId() == 23){
                return fiducial.getRobotPoseTargetSpace().getPosition().z;
            }
        }
        return Double.NaN; //what is Dave?
    }


    public void latchMotif() {
        ColorMatch.ArtifactColor[] current = getMotifPattern();
        if (current != null) {
            latchedMotif = current.clone();
        }
    }


    public ColorMatch.ArtifactColor[] getLatchedMotif() {
        return latchedMotif;
    }


}

