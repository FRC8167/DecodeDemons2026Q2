package org.firstinspires.ftc.teamcode.SubSystems;

import com.bylazar.configurables.annotations.Configurable;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.Robot;

@Configurable
public class Shooter extends SubsystemBase {
//        private final Robot robot = Robot.getInstance();

    public MotorEx shooterMotor;
        private final PIDFController shooterPID;
        private double ticksPerSec;  //PID uses ticksPerSecond

        //Panels Configurables
        public static double targetRPM = 0.0;
        public static double kv = 0.00045;
        public static double kp = 0.00045;
        public static double ki = 0.05;
        public static double kd = 0.00008;
        public static double tolerance= 100.0;  //RPMs




    public static final InterpLUT distanceToRPM;
        static{
            distanceToRPM = new InterpLUT();
            distanceToRPM.add(42.0, 2650.0);
            distanceToRPM.add(69.0, 2950.0);
            distanceToRPM.add(94.0, 3250.0);
            distanceToRPM.add(99.0, 3300.0);
            distanceToRPM.add(120.0, 3575.0);

            //in and RPM
            distanceToRPM.createLUT();
        }


        public Shooter(MotorEx motor) {
            shooterMotor = motor;
            shooterMotor.setRunMode(Motor.RunMode.VelocityControl);
            shooterMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            shooterMotor.setInverted(true);
            shooterPID = new PIDFController(kp, ki, kd, kv);
            shooterPID.setTolerance(convertRPMToTicksPerSec(tolerance));  // PID needs ticksPerSec
            setVelocity(0.0);
        }

        public void setTargetRPM(double rpm) {
            targetRPM = rpm;
            shooterPID.setSetPoint(rpm);
        }

        public void stop() {
            targetRPM = 0;
        }

        @Override
        public void periodic() {
            double output = 0;

            shooterPID.setPIDF(kp, ki, kd, kv);
            shooterPID.setTolerance(convertRPMToTicksPerSec(tolerance));

            if(shooterPID.getSetPoint() < 10)  {
                output = 0;
            } else {
                double currentVelocity = shooterMotor.getVelocity();
                output = shooterPID.calculate(currentVelocity, ticksPerSec);
            }
            shooterMotor.set(output);
        }

        public double convertRPMToTicksPerSec(double rpm) {
            return rpm * 28.0 / 60.0;
        }

        public double convertTicksPerSecToRPM(double ticksPerSec) {
            return ticksPerSec / 28.0 * 60.0;
        }

         public void setVelocity(double targetRPM) {
            ticksPerSec = convertRPMToTicksPerSec(targetRPM);
            shooterPID.setSetPoint(ticksPerSec);
        }

        public boolean atTargetVelocity() {
            return shooterPID.atSetPoint();
        }

        public double getTargetSpeed() {
            return shooterPID.getSetPoint();
        }

        public double getRPM() {
            return convertTicksPerSecToRPM(shooterMotor.getVelocity());
        }

        public void smartVelocity(double ATdistance) {
            double targetRPM = 0.0;
            if (ATdistance > 42 && ATdistance < 120){
                targetRPM = distanceToRPM.get(ATdistance);

            }
            else
            {
                targetRPM = 3575;
            }
            ticksPerSec = convertRPMToTicksPerSec(targetRPM);
            shooterPID.setSetPoint(ticksPerSec);
        }





    }

