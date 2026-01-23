package org.firstinspires.ftc.teamcode.SubSystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.Robot;

public class Intake extends SubsystemBase{

    private MotorEx intakeMotor;

    private boolean isRunning;

    private final double fSpeed = 0.75;
    private final double rSpeed = -0.5;
    private double speed = 0;
    public MotorState motorstate = MotorState.STOP;


    public Intake (MotorEx intakeMotor){
        this.intakeMotor = intakeMotor;
        intakeMotor.setInverted(true);
        stop();
    }

    public enum MotorState{
        REVERSE,
        STOP,
        FORWARD
};

   public void setMotorState(MotorState motorState)  {
       this.motorstate = motorState;
       setIntakeState();
   }


   public void setIntakePower(double power){
       this.speed = power;
       intakeMotor.set(power);
   }

   public void setIntakeState()  {
       switch(motorstate) {
           case FORWARD:
               intakeMotor.set(speed);
               break;
           case REVERSE:
               intakeMotor.set(-speed);
               break;
           case STOP:
               default:
                   intakeMotor.set(0.0);
                   break;

       }
   }


    public void stop(){
        intakeMotor.set(0.0);
        isRunning = false;
    }


    public void forward() {
        if (isRunning){
            stop();
        } else{
            intakeMotor.set(fSpeed);
            isRunning = true;}
    }


    public void reverse(){
        if (isRunning){
            stop();
        }else{
        intakeMotor.set(rSpeed);
        isRunning = true;}
    }

    public void intakeOn() {
        intakeMotor.set(fSpeed);
        isRunning = true;
    }

    public void intakeReverse() {
        intakeMotor.set(rSpeed);
        isRunning = true;
    }


    public boolean getIntakeState(){
        return isRunning;
    }
}






