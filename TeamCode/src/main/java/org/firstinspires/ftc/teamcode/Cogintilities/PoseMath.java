package org.firstinspires.ftc.teamcode.Cogintilities;

import androidx.annotation.NonNull;

//import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.ArrayList;
import java.util.List;

public class PoseMath {
    private PoseMath() {}

//    public static Pose2d Pose3DtoPose2d(Pose3D pose3D) {
//        if (pose3D == null) return null;
//        Position position = pose3D.getPosition().toUnit(DistanceUnit.INCH);
//        YawPitchRollAngles orientation = pose3D.getOrientation();
//
//        return new Pose2d(position.x, position.y, orientation.getYaw(AngleUnit.RADIANS));
//    }

    static public <T,T2> double pose3DDistance(T firstPose, T2 secondPose) {
        Position firstPosition = normalizeToPosition(firstPose);
        Position secondPosition = normalizeToPosition(secondPose);
        if (firstPosition == null || secondPosition == null) return Double.NaN;
        double x1 = firstPosition.x;
        double y1 = firstPosition.y;
        double z1 = firstPosition.z;
        double x2 = secondPosition.x;
        double y2 = secondPosition.y;
        double z2 = secondPosition.z;
        return Math.sqrt(Math.pow(x2-x1,2) + Math.pow(y2-y1,2) + Math.pow(z2-z1,2));
    }

    static public <T,T2> double poseFlattenedDistance(T firstPose, T2 secondPose, DistanceUnit distanceUnit) {
        Position firstPosition = normalizeToPosition(firstPose).toUnit(distanceUnit);
        Position secondPosition = normalizeToPosition(secondPose).toUnit(distanceUnit);
        if (firstPosition == null || secondPosition == null) return Double.NaN;
        double x1 = firstPosition.x;
        double y1 = firstPosition.y;
        double x2 = secondPosition.x;
        double y2 = secondPosition.y;
        return Math.sqrt(Math.pow(x2-x1,2) + Math.pow(y2-y1,2));
    }

    static public <T> double pose3DDistance(T pose) {
        Position originPosition = new Position(DistanceUnit.INCH,0,0,0,0);
        return pose3DDistance(originPosition, pose);
    }

    static public <T> Position normalizeToPosition(T pose) {
        if (pose == null) return null;
        else if (pose instanceof Pose3D) return ((Pose3D) pose).getPosition();
        else if (pose instanceof Position) return (Position) pose;
        else return null;
    }

    static public <T,T2> double poseArcTan(T firstPose, T2 secondPose, AngleUnit unit) {
        Position firstPosition = normalizeToPosition(firstPose);
        Position secondPosition = normalizeToPosition(secondPose);
        if (firstPosition == null || secondPosition == null) return Double.NaN;
        double x1 = firstPosition.x;
        double y1 = firstPosition.y;
        double x2 = secondPosition.x;
        double y2 = secondPosition.y;
        double radianAngle = Math.atan2(y2-y1, x2-x1);
        return unit.fromRadians(radianAngle);
    }

    static public <T> double poseArcTan(T pose, AngleUnit unit) {
        Position originPosition = new Position(DistanceUnit.INCH,0,0,0,0);
        return poseArcTan(originPosition, pose, unit);
    }

    static public Pose3D poseMedian(List<Pose3D> poseList, DistanceUnit distanceUnit, AngleUnit angleUnit) {
        if (poseList == null || poseList.isEmpty()) return null;
        List<Double> x = new ArrayList<>();
        List<Double> y = new ArrayList<>();
        List<Double> z = new ArrayList<>();
        List<Double> yaw = new ArrayList<>();
        List<Double> pitch = new ArrayList<>();
        List<Double> roll = new ArrayList<>();
        for (Pose3D pose : poseList) {
            Position position = pose.getPosition().toUnit(distanceUnit);
            YawPitchRollAngles orientation = pose.getOrientation();
            x.add(position.x);
            y.add(position.y);
            z.add(position.z);
            yaw.add(orientation.getYaw(angleUnit));
            pitch.add(orientation.getPitch(angleUnit));
            roll.add(orientation.getRoll(angleUnit));
        }

        double newX = VariousMath.median(x);
        double newY = VariousMath.median(y);
        double newZ = VariousMath.median(z);
        double newYaw = VariousMath.median(yaw);
        double newPitch = VariousMath.median(pitch);
        double newRoll = VariousMath.median(roll);

        Position newPosition = new Position(distanceUnit, newX, newY, newZ, System.nanoTime());
        YawPitchRollAngles newOrientation = new YawPitchRollAngles(angleUnit, newYaw, newPitch, newRoll, System.nanoTime());

        return new Pose3D(newPosition, newOrientation);
    }

    static public List<Pose3D> removeOldPoses(List<Pose3D> poseList, double stalenessSeconds) {
        if (poseList == null) return null;
        if (poseList.size() <=1) return poseList;

        poseList.removeIf(pose3D -> poseStaleness(pose3D) > stalenessSeconds);
        return poseList;
    }

    @NonNull
    static public Pose3D poseWithAcquisitionTime(@NonNull Pose3D pose3D, long acquisitionTime) {
        Position position = pose3D.getPosition();
        YawPitchRollAngles orientation = pose3D.getOrientation();
        Position newPosition = new Position(position.unit, position.x, position.y, position.z, acquisitionTime);
        YawPitchRollAngles newOrientation = new YawPitchRollAngles(
                AngleUnit.DEGREES,
                orientation.getYaw(AngleUnit.DEGREES),
                orientation.getPitch(AngleUnit.DEGREES),
                orientation.getRoll(AngleUnit.DEGREES),
                acquisitionTime);
        return new Pose3D(newPosition, newOrientation);
    }

    @NonNull
    static public Pose3D poseWithAcquisitionTime(@NonNull Pose3D pose3D) {
        return poseWithAcquisitionTime(pose3D, System.nanoTime());
    }

    static public double poseStaleness(Pose3D pose3D) {
        double positionStaleness    = VariousMath.elapsedSecondsFromAbsoluteNano(pose3D.getPosition().acquisitionTime);
        double orientationStaleness = VariousMath.elapsedSecondsFromAbsoluteNano(pose3D.getOrientation().getAcquisitionTime());
        return Math.max(positionStaleness, orientationStaleness);
    }

    static public <T> boolean poseIsValid(T pose) {
        Position position = normalizeToPosition(pose);
        if (position == null) return false;
        return !(position.x > 72 || position.x < -72 || position.y > 72 || position.y < -72 || position.z > 10 || position.z < -10);
    }

}
