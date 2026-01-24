package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.SubSystems.Shooter;

public class ShooterSpinUpCommand extends CommandBase {

    private final Shooter shooter;
    private final double targetRPM;

    public ShooterSpinUpCommand(Shooter shooterSubsystem, double targetRPM) {
        this.shooter= shooterSubsystem;
        this.targetRPM = targetRPM;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
//        shooterSubsystemTest.setVelocity(0.0);
        shooter.setVelocity(targetRPM);

    }


    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return shooter.atTargetVelocity();
    }

    @Override
    public void end(boolean interrupted) {
//            shooter.stop();

    }
}
