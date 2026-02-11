package org.firstinspires.ftc.teamcode.SubSystems;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

@Configurable
public class Slide extends SubsystemBase {

    private final MotorEx slideMotor;
    public static double MAX_POWER = 0.75 ;
    public static final double TICKS_PER_REV = 537.7;
    public static final double MM_PER_REV = 120.0;
    public static final double TICKS_PER_MM = TICKS_PER_REV / MM_PER_REV;//4.5
    public static int NEST_POS = 8;
    public static int KICK_POS = 460;  //units are ticks

    //Limits
    public static int DOWN_LIMIT = 5;
    public static int UP_LIMIT   = 470;  //units are ticks

    // PID using ticks
    public static double kP = 0.003; //0.0155;
    public static double kI = 0.0;
    public static double kD = 0; //2.5E-4;
    public static double kF = 0.0;

    public static double TOLERANCE = 10;
    private final PIDFController slidePID;
    private int targetTicks = 0;


    public Slide(MotorEx slideMotor) {
        this.slideMotor = slideMotor;
        slideMotor.setRunMode(MotorEx.RunMode.RawPower);
        slideMotor.resetEncoder();
        slideMotor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        slidePID = new PIDFController(kP, kI, kD, kF);
        slidePID.setTolerance(TOLERANCE);
        //setTargetTicks(NEST_POS);
    }


    public void setTargetTicks(int ticks) {
        targetTicks = clamp(ticks);
        slidePID.setSetPoint(targetTicks);
    }


    @Override
    public void periodic() {
//        slidePID.setPIDF(kP, kI, kD, kF);
//        slidePID.setTolerance(TOLERANCE);
        double currentPosition = slideMotor.getCurrentPosition();
        double output = slidePID.calculate(currentPosition);
        output = Range.clip(output, -MAX_POWER, MAX_POWER);
        slideMotor.set(output);
    }


    public void stop() {
        slidePID.setSetPoint(NEST_POS);
    }


    private int clamp(int ticks) {
        if (ticks > UP_LIMIT) return UP_LIMIT;
        else if (ticks < DOWN_LIMIT) return DOWN_LIMIT;
        return ticks;
    }


    public int getPositionTicks() {
        return slideMotor.getCurrentPosition();
    }

    public int getPidSetpoint() { return (int)slidePID.getSetPoint(); }

    public boolean atTarget() {
        return slidePID.atSetPoint();
    }

}
