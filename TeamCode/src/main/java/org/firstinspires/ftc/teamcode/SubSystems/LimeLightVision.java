package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Cogintilities.PoseMath;
import org.firstinspires.ftc.teamcode.Cogintilities.TeamConstants;

import java.util.ArrayList;
import java.util.List;

public class LimeLightVision extends SubsystemBase implements TeamConstants {
    double LIME_VISION_STALENESS_TOL = 200; //Staleness in milliseconds
    double LIME_VISION_POSE_MEDIATING_STALENESS_TOL = 0.5; //Staleness in seconds for which older poses will be removed
    double LIME_VISION_POSE_MEDIATING_MIN_POSES = 3;

    Position BLUE_GOAL_TARGET = new Position(DistanceUnit.INCH, -60.699807, -57.531366, 38.75, 0);
    Position RED_GOAL_TARGET = new Position(DistanceUnit.INCH, -60.699807, 57.531366, 38.75, 0);

    private static ColorMatch.ArtifactColor[] latchedMotif = null;

    private final Limelight3A limelight;
//    IMU imu;
    private int pipeline;

    private List<Pose3D> previousPoses = new ArrayList<>();
    private double previousTagID;

    public LimeLightVision(Limelight3A limelight) {
        this.limelight = limelight;
        this.limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        this.limelight.start(); // This tells Limelight to start looking!
        this.limelight.pipelineSwitch(0);
        pipeline = 0;
    }

    public void resetMotif() {
        latchedMotif = null;
    }


    public void setPipeline(int pipeline) {
        this.limelight.pipelineSwitch(pipeline);
        this.pipeline = pipeline;
    }

    public void start() {
        limelight.start();
    }

    public void stop() {
        limelight.stop();
    }

    public LLResult getResult() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid() && result.getPipelineIndex() == pipeline && result.getStaleness() <= LIME_VISION_STALENESS_TOL) {
            return result;
        } else return null;
    }

    public Pose3D getRobotPose3D() {
        previousPoses = PoseMath.removeOldPoses(previousPoses, LIME_VISION_POSE_MEDIATING_STALENESS_TOL);
        LLResult result = getResult();
        if (result == null) return null;
        List<LLResultTypes.FiducialResult> fiducials = getGoalFiducials(result);
        double fiducialID;
        if (fiducials != null && !fiducials.isEmpty()) {
            fiducialID = fiducials.get(0).getFiducialId();
            if (fiducialID != previousTagID) {
                previousPoses.clear();
                previousTagID = fiducialID;
            }
        } else return null;

        Pose3D pose = result.getBotpose();
        if (pose == null || !PoseMath.poseIsValid(pose)) return null;
        previousPoses.add(PoseMath.poseWithAcquisitionTime(pose));
        return pose;
    }

    public Pose3D getMediatiatedRobotPose3D() {
        getRobotPose3D();
        if (previousPoses.size() < LIME_VISION_POSE_MEDIATING_MIN_POSES) return null;
        return PoseMath.poseMedian(previousPoses, DistanceUnit.INCH, AngleUnit.DEGREES);
    }

    public double getPreviousPosesSize() {
        return previousPoses.size();
    }

//    public Pose2d getRobotPose2d() {
//        return PoseMath.Pose3DtoPose2d(getRobotPose3D());
//    }
//
//    public Pose2d getMediatedRobotPose2d() {
//        return PoseMath.Pose3DtoPose2d(getMediatiatedRobotPose3D());
//    }

    public double getGoalBearing(LLResult result, Pose3D pose3D) {
        if (result == null) return Double.NaN;
        if (pose3D == null) return Double.NaN;
        Position position = pose3D.getPosition().toUnit(DistanceUnit.INCH);
        YawPitchRollAngles orientation = pose3D.getOrientation();

        List<LLResultTypes.FiducialResult> goalFiducials = getGoalFiducials(result);
        if (goalFiducials == null || goalFiducials.isEmpty()) return Double.NaN;
        LLResultTypes.FiducialResult goalFiducial = null;
        if (goalFiducials.size() == 1) {
            goalFiducial = goalFiducials.get(0);
        } else {
            for (LLResultTypes.FiducialResult fiducialResult : goalFiducials) {
                int id = fiducialResult.getFiducialId();
                double angle = orientation.getYaw(AngleUnit.DEGREES);
                if (angle >= -180 && angle < 0 && id == 20) {
                    goalFiducial = fiducialResult;
                    break;
                } else if (angle >= 0 && angle < 180 && id == 24) {
                    goalFiducial = fiducialResult;
                    break;
                }

            }
        }

        if (goalFiducial == null) return Double.NaN;

        double rawAngle;

        int id = goalFiducial.getFiducialId();
        if (id == 20) {
            rawAngle = PoseMath.poseArcTan(position, BLUE_GOAL_TARGET, AngleUnit.DEGREES);
        } else if (id == 24) {
            rawAngle = PoseMath.poseArcTan(position, RED_GOAL_TARGET, AngleUnit.DEGREES);
        } else {
            return Double.NaN;
        }

        double robotBearing = rawAngle-orientation.getYaw(AngleUnit.DEGREES);
        return AngleUnit.normalizeDegrees(robotBearing);
    }

    public double getGoalBearing(Pose3D pose3D) {
        return getGoalBearing(getResult(), pose3D);
    }

    public double getGoalBearing() {
        return getGoalBearing(getResult(), getRobotPose3D());
    }

    public double getMediatedGoalBearing() {
        return getGoalBearing(getResult(), getMediatiatedRobotPose3D());
    }

    public double getGoalDistance(LLResult result, Pose3D pose3D) {
        if (result == null) return Double.NaN;
        if (pose3D == null) return Double.NaN;
        Position position = pose3D.getPosition().toUnit(DistanceUnit.INCH);
        YawPitchRollAngles orientation = pose3D.getOrientation();

        List<LLResultTypes.FiducialResult> goalFiducials = getGoalFiducials(result);
        if (goalFiducials == null || goalFiducials.isEmpty()) return Double.NaN;
        LLResultTypes.FiducialResult goalFiducial = null;
        if (goalFiducials.size() == 1) {
            goalFiducial = goalFiducials.get(0);
        } else {
            for (LLResultTypes.FiducialResult fiducialResult : goalFiducials) {
                int id = fiducialResult.getFiducialId();
                double angle = orientation.getYaw(AngleUnit.DEGREES);
                if (angle >= -180 && angle < 0 && id == 20) {
                    goalFiducial = fiducialResult;
                    break;
                } else if (angle >= 0 && angle < 180 && id == 24) {
                    goalFiducial = fiducialResult;
                    break;
                }

            }
        }

        if (goalFiducial == null) return Double.NaN;

        double distance;

        int id = goalFiducial.getFiducialId();
        if (id == 20) {
            distance = PoseMath.poseFlattenedDistance(position, BLUE_GOAL_TARGET, DistanceUnit.INCH);
        } else if (id == 24) {
            distance = PoseMath.poseFlattenedDistance(position, RED_GOAL_TARGET, DistanceUnit.INCH);
        } else {
            return Double.NaN;
        }

        return distance;
    }

    public double getGoalDistance() {
        return getGoalDistance(getResult(), getRobotPose3D());
    }

    public double getMediatedGoalDistance() {
        return getGoalDistance(getResult(), getMediatiatedRobotPose3D());
    }

    static public List<LLResultTypes.FiducialResult> getGoalFiducials(LLResult result) { //Note: O
        if (result == null || !result.isValid()) return null;
        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
        if (fiducialResults == null || fiducialResults.isEmpty()) return null;
        List<LLResultTypes.FiducialResult> goalResults = new ArrayList<>();
        for (LLResultTypes.FiducialResult fiducialResult : fiducialResults) {
            int id = fiducialResult.getFiducialId();
            if (id == 20 || id == 24) {
                goalResults.add(fiducialResult);
            }
        }
        return goalResults;
    }

    static public List<LLResultTypes.FiducialResult> getObeliskFiducials(LLResult result) { //Note: O
        if (result == null || !result.isValid()) return null;
        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
        if (fiducialResults == null || fiducialResults.isEmpty()) return null;
        List<LLResultTypes.FiducialResult> obeliskResults = new ArrayList<>();
        for (LLResultTypes.FiducialResult fiducialResult : fiducialResults) {
            int id = fiducialResult.getFiducialId();
            if (id == 21 || id == 22 || id == 23) {
                obeliskResults.add(fiducialResult);
            }
        }
        return obeliskResults;
    }

    static public List<Integer> getObeliskIDs(LLResult result) { //Note: O
        List<LLResultTypes.FiducialResult> obeliskResults = getObeliskFiducials(result);
        if (obeliskResults == null || obeliskResults.isEmpty()) return null;
        List<Integer> idList = new ArrayList<>();
        for (LLResultTypes.FiducialResult fiducialResult : obeliskResults) {
            idList.add(fiducialResult.getFiducialId());
        }
        return idList;
    }

    public ColorMatch.ArtifactColor[] getIdStates(int id) {
        switch (id) {
            case 21: return new ColorMatch.ArtifactColor[]{ColorMatch.ArtifactColor.GREEN, ColorMatch.ArtifactColor.PURPLE, ColorMatch.ArtifactColor.PURPLE};
            case 22: return new ColorMatch.ArtifactColor[]{ColorMatch.ArtifactColor.PURPLE, ColorMatch.ArtifactColor.GREEN, ColorMatch.ArtifactColor.PURPLE};
            case 23: return new ColorMatch.ArtifactColor[]{ColorMatch.ArtifactColor.PURPLE, ColorMatch.ArtifactColor.PURPLE, ColorMatch.ArtifactColor.GREEN};
            default: return null;
        }
    }

    public List<ColorMatch.ArtifactColor[]> getSequences() {
        List<Integer> obeliskIDs = getObeliskIDs(getResult());
        if (obeliskIDs == null || obeliskIDs.isEmpty()) return null;
        List<ColorMatch.ArtifactColor[]> sequences = new ArrayList<>();
        for (Integer ID : obeliskIDs) {
            ColorMatch.ArtifactColor[] states = getIdStates(ID);
            if (states != null) sequences.add(states);
        }
        return sequences.isEmpty() ? null : sequences;
    }

    public ColorMatch.ArtifactColor[] getFirstSequence() {
        List<ColorMatch.ArtifactColor[]> sequences = getSequences();
        return (sequences == null || sequences.isEmpty()) ? null : sequences.get(0);
    }

    static public String tagIdLookup(int id) {
        switch (id) {
            case 20: return "BLUE GOAL";
            case 21: return "GPP";
            case 22: return "PGP";
            case 23: return "PPG";
            case 24: return "RED GOAL";
            default: return "Unknown";
        }
    }

    public void latchMotif() {
        ColorMatch.ArtifactColor[] current = getFirstSequence();
        if (current != null) {
            latchedMotif = current.clone();
        }
    }


    public ColorMatch.ArtifactColor[] getLatchedMotif() {
        return latchedMotif;
    }

//    public Pose3D getMT2Pos() {
//        double robotYaw = imu.getRobotYawPitchRollAngles().getYaw();
//        limelight.updateRobotOrientation(robotYaw);
//        LLResult result = getResult();
//        if (result == null) return null;
//        return result.getBotpose_MT2();
//    }
public double getTX() {
    LLResult result = getResult();
    if (result != null && result.isValid()) {
        // Priority: Check if the goal fiducials are being tracked specifically
        List<LLResultTypes.FiducialResult> goals = getGoalFiducials(result);
        if (goals != null && !goals.isEmpty()) {
            // Return the Tx of the first tracked Goal tag
            //return goals.get(0).getTX();

        }
        // Fallback: Return the Tx of whatever the primary target is
        return result.getTx();
    }
    return 0.0;
}

    /**
     * Checks if Limelight sees a valid target to align to.
     */
    public boolean hasTarget() {
        LLResult result = getResult();
        return result != null && result.isValid();
    }

    public void takePhoto(String string) {
        limelight.captureSnapshot(string);
    }




}
