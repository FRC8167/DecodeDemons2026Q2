package org.firstinspires.ftc.teamcode.SubSystems;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.command.SubsystemBase;


@Configurable
public class Slide extends SubsystemBase {

    private final DcMotorEx slideMotor;

    // Increased power for faster kicking
    // public static double KICK_MAX_POWER = 0.8;
    // public static double NEST_MAX_POWER = 0.5;

    // public static final double TICKS_PER_REV = 537.7;
    // public static final double MM_PER_REV = 120.0;
    // public static final double TICKS_PER_MM = TICKS_PER_REV / MM_PER_REV; //4.5
    public static int NEST_POS = 8;
    public static int KICK_POS = 460;  //units are ticks
    public static double RATE = Math.abs(KICK_POS - NEST_POS) / 0.5; // Travel full range in 0.5s
    // Equates to 904 ticks/sec * 1rev/537.7 ticks * 60s/1min = 101rpm (315 rpm motor installed)
    // fastest theoretical rate: 2822 tick/sec

    //Limits
    public static int DOWN_LIMIT = 5;
    public static int UP_LIMIT   = 470;  //units are ticks

    // PID using ticks
    public static double kp= 0.0035; //.003
//    public static double ki = 0.0;
//    public static double kd= .00018; //.00015
//    public static double kf = 0.0;
    public static double TOLERANCE = 6;
    private PIDFCoefficients coeffs;


    public Slide(DcMotorEx slideMotor) {
        this.slideMotor = slideMotor;
        slideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        slideMotor.setDirection(DcMotorEx.Direction.FORWARD);
        slideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setPositionPIDFCoefficients(kp);
        //setTargetTicks(NEST_POS);
    }

    public void kick() {
        moveToSetPoint(KICK_POS);
    }

    public void nest() {
        moveToSetPoint(NEST_POS);
    }

    private void moveToSetPoint(int setPointCounts) {

        double trgtPosCnts = clamp(setPointCounts);

        /** https://docs.revrobotics.com/duo-control/programming/using-encoder-feedback#choosing-a-motor-mode
         *  https://ftctechnh.github.io/ftc_app/doc/javadoc/com/qualcomm/robotcore/hardware/DcMotorEx.html
         *
         *  DcMotorEx method RUN_TO_POSITION must be done in the following order:
         *      1. Set target position [encoder counts]
         *      2. Set motor mode to RUN_TO_POSITION
         *      3. Set the maximum velocity or power you want the motor to use
         *          motor.setVelocity(rate) [counts/sec]
         *          motor.setVelocity(rate, units) [ AngleUnit.DEGREES/sec or AngleUnit.RADIANS/sec]
         *          motor.setPower(%); 0.0 - 1.0
         */

        slideMotor.setTargetPosition(clamp((int)trgtPosCnts));
        slideMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        slideMotor.setVelocity(RATE);
    }


    @Override
    public void periodic() {
        /** For tuning purposes only! Comment/Delete for competition */
        if(!slideMotor.isBusy()){
            slideMotor.setPositionPIDFCoefficients(kp);
            slideMotor.setTargetPositionTolerance((int)TOLERANCE);
        }
    }


    public void stop() {
        slideMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        slideMotor.setPower(0);
    }


    private int clamp(int ticks) {
        int setPt = ticks;
        if      (ticks > UP_LIMIT)   setPt = UP_LIMIT;
        else if (ticks < DOWN_LIMIT) setPt = DOWN_LIMIT;
        return setPt;
    }


    public void setTargetTicks(int targetCount) {
        moveToSetPoint(targetCount);
    }


    public boolean atTarget() {
        return !slideMotor.isBusy();
        /* OR
        int error = Math.abs(slideMotor.getCurrentPosition() - slideMotor.getTargetPosition());
        return (error < POS_TOLERANCE) ? true : false;
        */
    }


    public int getPositionTicks() {
        return slideMotor.getCurrentPosition();
    }


    public int getPidSetpoint() { return (int)slideMotor.getTargetPosition(); }



}