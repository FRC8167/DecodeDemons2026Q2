package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    //mass of 15 is a placeholder
    public static FollowerConstants followerConstants = new FollowerConstants()
            .forwardZeroPowerAcceleration(-29.29619849002874)
            .lateralZeroPowerAcceleration(-56.40418997715789)
            .mass(5.3)

            //OLD ROBOT VALUES
//            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, .03, 0.025))
//            .headingPIDFCoefficients(new PIDFCoefficients(0.7, 0.0, 0.002, 0.025))
//            .drivePIDFCoefficients(new FilteredPIDFCoefficients(.006, 0.0, .0006, 0.6, 0.01))

//TUES VALS
//            .translationalPIDFCoefficients(new PIDFCoefficients(0.17, 0, 0.0, 0.025))
//            .headingPIDFCoefficients(new PIDFCoefficients(0.1, 0.0, 0.0, 0.0))//.7
//            .drivePIDFCoefficients(new FilteredPIDFCoefficients(.006, 0.0, .0006, 0.6, 0.007))

//SOME PEDRO TEAM
//            .translationalPIDFCoefficients(new PIDFCoefficients(0.04, 0, 0.004, 0.03))
//            .headingPIDFCoefficients(new PIDFCoefficients(.45, 0, 0.02, .03))
//            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.06, 0, 0.0006, 0.6, 0.02))

//            .centripetalScaling(.0005)


            .translationalPIDFCoefficients(new PIDFCoefficients(0.08, 0.0, 0.008, 0.005))
            .headingPIDFCoefficients(new PIDFCoefficients(0.7, 0.0, 0.01, 0.02))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(.006, 0.0, .0006, 0.6, 0.007))
            .centripetalScaling(0.0003);






//    public static PathConstraints pathConstraints = new PathConstraints(
    public static PathConstraints pathConstraints = new PathConstraints(
        0.99,   // max power?
        100,     // max velocity?
        0.5,    // accel?
        1.0     // decel?
);






    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1.0)
            .xVelocity(85.35567059854823)
            .yVelocity(70.77068611205094)
            .rightFrontMotorName("RightFront")  //TODO check these names                                             c
            .rightRearMotorName("RightRear")
            .leftRearMotorName("LeftRear")
            .leftFrontMotorName("LeftFront")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(0.0)  //TODO
            .strafePodX(-6.5)  //TODO
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")  //change name?
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)  //TODO verify or REVERSE
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);  //TODO verify or REVERSE
}
