package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.SubSystems.Juggler;
import org.firstinspires.ftc.teamcode.SubSystems.Popper;


public class SinglePopCommand extends SequentialCommandGroup {

    public SinglePopCommand(Popper popper) {
        addRequirements(popper);

        addCommands(
                new InstantCommand(() -> popper.set(Popper.PopperState.KICK)),
                new WaitCommand(350), //was 500
                new InstantCommand(() -> popper.set(Popper.PopperState.RESET)),
                new WaitCommand(350)

        );
    }




//    @Override
//    public void end(boolean interrupted) {
//
//    }
}
