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
        public static double tolerance= 20.0;  //RPMs


    public static final InterpLUT distanceToRPM;
        static{
            distanceToRPM = new InterpLUT();
            distanceToRPM.add(41.0, 2900);
            distanceToRPM.add(45.0, 2950);
            distanceToRPM.add(70.0, 2975);
            distanceToRPM.add(87.0, 3165);
            distanceToRPM.add(115.0, 3475);
            distanceToRPM.add(123.0, 3775);
            distanceToRPM.add(138.0, 3900);

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
            shooterPID.setSetPoint(0.0);
        }

        @Override
        public void periodic() {
            double output = 0;

            shooterPID.setPIDF(kp, ki, kd, kv);
            shooterPID.setTolerance(convertRPMToTicksPerSec(tolerance));

            double currentVelocity = shooterMotor.getVelocity();
            if(shooterPID.getSetPoint() < 1)  {
                output = 0;
            } else {
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

        public void smartVelocity(double ATdistance, ColorMatch.ArtifactColor color) {
            double targetRPM = 0.0;
            if (ATdistance > 41 && ATdistance < 138){
                if (color == ColorMatch.ArtifactColor.PURPLE || color == ColorMatch.ArtifactColor.UNKNOWN)
                    {
                    targetRPM=0.11*ATdistance*ATdistance-8.72*ATdistance+3102.92;
                    }
                    //targetRPM = distanceToRPM.get(ATdistance);
                else{
                    targetRPM=0.11*ATdistance*ATdistance-8.72*ATdistance+3102.92 + 75;  //TODO define DELTA
                }

            }
            else
            {
                targetRPM = 3775;
            }
            ticksPerSec = convertRPMToTicksPerSec(targetRPM);
            shooterPID.setSetPoint(ticksPerSec);
        }





    }

