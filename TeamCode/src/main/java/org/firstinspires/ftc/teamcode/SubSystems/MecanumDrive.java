package org.firstinspires.ftc.teamcode.SubSystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

/**
 * MecanumDrive Subsystem
 * Handles low-level motor control and odometry updates.
 * Operator   input handled in a separate DriveCommand.
 */
@Configurable
public class  MecanumDrive extends SubsystemBase {

    private final MotorEx frontLeft, backLeft, frontRight, backRight;

    private double controlAuthority;
    private final double DEGRADE_AUTHORITY = 0.35;
    private final double MAX_AUTHORITY = 0.75;

    public enum DriveMode {
        NORMAL,
        CONSTANT_HEADING,
        FIELD_ORIENTATION
    }

    private double robotHeading, targetBearing;
    private boolean targetValid;
    private DriveMode driveMode;

    // Needed for Panels Tuning - Can be local variable values known
    double Kp_Heading = 0.1;
    double FFheading = 0;


    /**
     *
     * @param leftFront
     * @param leftRear
     * @param rightFront
     * @param rightRear
     */
    public MecanumDrive(MotorEx leftFront, MotorEx leftRear, MotorEx rightFront, MotorEx rightRear) {

        this.frontLeft = leftFront;
        this.backLeft = leftRear;
        this.frontRight = rightFront;
        this.backRight = rightRear;

        // Set motor directions
        frontLeft.setInverted(true);
        backLeft.setInverted(true);
        frontRight.setInverted(false);
        backRight.setInverted(false);

        frontRight.setRunMode(Motor.RunMode.RawPower);
        frontLeft.setRunMode(Motor.RunMode.RawPower);
        backLeft.setRunMode(Motor.RunMode.RawPower);
        backRight.setRunMode(Motor.RunMode.RawPower);

        controlAuthority = MAX_AUTHORITY;
        driveMode = DriveMode.NORMAL;
        targetValid = false;

        // Initialize motors
        setMotorPower(0, 0, 0, 0);
    }


    /**
     * Apply mecanum powers
     *
     * @param driveCmd  forward/back
     * @param strafeCmd left/right
     * @param turnCmd   rotation
     */
    public void calcDriveVals(double driveCmd, double strafeCmd, double turnCmd) {

//        double drive  = driveCmd  * controlAuthority;
//        double strafe = strafeCmd * controlAuthority;
//        double turn   = turnCmd   * controlAuthority;
//
//        double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(turn), 1);
//
//        double frontLeftPower  = (drive + strafe + turn) / denominator;
//        double backLeftPower   = (drive - strafe + turn) / denominator;
//        double frontRightPower = (drive - strafe - turn) / denominator;
//        double backRightPower  = (drive + strafe - turn) / denominator;
//
//        setMotorPower(frontLeftPower, frontRightPower, backLeftPower, backRightPower);

        /* New on 1/25 - Drive Mode Addition */
        double denominator = Math.max(Math.abs(driveCmd) + Math.abs(strafeCmd) + Math.abs(turnCmd), 1);

        double frontLeftPower  = (driveCmd + strafeCmd + turnCmd) / denominator;
        double backLeftPower   = (driveCmd - strafeCmd + turnCmd) / denominator;
        double frontRightPower = (driveCmd - strafeCmd - turnCmd) / denominator;
        double backRightPower  = (driveCmd + strafeCmd - turnCmd) / denominator;

        setMotorPower(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }


    public void drive(double driveCmd, double strafeCmd, double turnCmd){

        double error, newTurnCmd;
        double drive, strafe, turn;

        switch (driveMode) {

            case NORMAL:
                drive  = driveCmd  * controlAuthority;
                strafe = strafeCmd * controlAuthority;
                turn   = turnCmd   * controlAuthority;
                calcDriveVals(drive, strafe, turn);
                break;

            case CONSTANT_HEADING:
                if(targetValid) {
                    error = targetBearing - robotHeading;

                    /* Need Angle Wrap calculation to ensure turning the shortest distance */
                    while (error > 180) {
                        error -= 2 * 180;
                    }
                    while (error < -180) {
                        error += 2 * 180;
                    }

                    newTurnCmd = Kp_Heading * error;
                    newTurnCmd += (newTurnCmd > 0) ? FFheading : -FFheading;
                    newTurnCmd = Range.clip(newTurnCmd, -1.0, 1.0);

                } else {
                    newTurnCmd = turnCmd;
                }

                calcDriveVals(driveCmd, strafeCmd, newTurnCmd);
                break;

            case FIELD_ORIENTATION:
                break;

        }
    }


    /**
     * Drive with a constant heading
     *
     * @param driveCmd          Drive command (FWD/REV) from Joystick Input
     * @param strafeCmd         Strafe command from Joystick Input
     * @param turnCmd           Turn command from Joystick Input
     * @param cmdHeadingDeg     Desired Heading to Maintain
     * @param currentHeading    Robots Current Heading
     */
    public void driveWithHeading(double driveCmd, double strafeCmd, double turnCmd, double cmdHeadingDeg, double currentHeading) {

        double error, newTurnCmd;
        double Kp_Heading = 0.1;
        double feedForwardGain = 0;

        error = cmdHeadingDeg - currentHeading;

        /* Need Angle Wrap calculation to ensure turning the shortest distance */
        while (error > 180) {
            error -= 2 * 180;
        }
        while (error < -180) {
            error += 2 * 180;
        }

        if (error >= 10 && error < 30) {     // Limit PID control range to +/- 30 degrees of target to eliminate angle wrap
            newTurnCmd = Kp_Heading * error;
        } else newTurnCmd = turnCmd;

        newTurnCmd += (newTurnCmd > 0) ? feedForwardGain : -feedForwardGain;

        drive(driveCmd, strafeCmd, newTurnCmd);
    }


    /**
     * his routine drives the robot field relative
     *
     * @param forward
     * @param right
     * @param rotate
     * @param currentHeading
     */
    public void driveFieldRelative(double forward, double right, double rotate, double currentHeading) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta - currentHeading);
//                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate);
    }


    /**
     * Low-level method: set motor power directly
     */
    private void setMotorPower(double lf, double rf, double lr, double rr) {
        frontLeft.set(lf);
        frontRight.set(rf);
        backLeft.set(lr);
        backRight.set(rr);
    }


    @Override
    public void periodic() {

    }


    public void enableSnailDrive() {
        controlAuthority = DEGRADE_AUTHORITY;
    }

    public void disableSnailDrive() {
        controlAuthority = MAX_AUTHORITY;
    }

    public void setDriveMode(DriveMode driveMode) {
        this.driveMode = driveMode;
        if(this.driveMode == DriveMode.NORMAL) targetValid = false;
    }

    // Needed for Panels Tuning
    public void setKpHeading(double kp){ Kp_Heading = kp; }
    public void setFFheading(double FF){ FFheading  = FF; }


    public void setBearings(double robotPoseYaw, double targetPose) {
        this.robotHeading = robotPoseYaw;
        if(targetPose != -999)  {
            this.targetBearing = targetPose;
            targetValid = true;
        } else targetValid = false;
    }

    public double getControlAuthority() {
        return controlAuthority;
    }

}
