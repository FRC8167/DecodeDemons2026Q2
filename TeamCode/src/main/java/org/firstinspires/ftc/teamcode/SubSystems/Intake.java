package org.firstinspires.ftc.teamcode.SubSystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.Robot;

public class Intake extends SubsystemBase{

    private MotorEx intakeMotor;

    private boolean isRunning;

    private Double fSpeed = 0.75;
    private Double rSpeed = -0.5;
    public MotorState motorstate = MotorState.STOP;


    public Intake (MotorEx intakeMotor){
        this.intakeMotor = intakeMotor;
        intakeMotor.setInverted(true);
        off();
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

   public void setIntakeState()  {
       switch(motorstate) {
           case FORWARD:
               intakeMotor.set(fSpeed);
               break;
           case REVERSE:
               intakeMotor.set(rSpeed);
               break;
           case STOP:
               default:
                   intakeMotor.set(0.0);
                   break;

       }
   }


    public void off(){
        intakeMotor.set(0.0);
        isRunning = false;
    }


    public void forward() {
        if (isRunning){
            off();
        } else{
            intakeMotor.set(fSpeed);
            isRunning = true;}
    }


    public void reverse(){
        if (isRunning){
            off();
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






