package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.SubSystems.Intake;

public class IntakeCommand extends CommandBase {
    private final Intake intake;
    private final Intake.MotorState motorState;
    private final ElapsedTime timer = new ElapsedTime();
    private final double duration;
    private final double power;

    public IntakeCommand(Intake intake, Intake.MotorState motorState, double duration, double power){
        this.intake = intake;
        this.motorState = motorState;
        this.duration = duration;
        this.power = power;
        addRequirements(intake);
    }


    @Override
    public void initialize() {
        timer.reset();
        intake.setMotorState(motorState);
        intake.setIntakePower(power);
        intake.setIntakeState();
    }

    @Override
    public void execute() {
        intake.setIntakeState();
    }

    @Override
    public boolean isFinished() {
        return duration > 0 && timer.milliseconds() > duration;
    }


    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }

}

