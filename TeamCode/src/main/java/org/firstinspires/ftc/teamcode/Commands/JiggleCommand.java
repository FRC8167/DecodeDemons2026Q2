//package org.firstinspires.ftc.teamcode.Commands;
//
//import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
//import com.seattlesolvers.solverslib.command.InstantCommand;
//import com.seattlesolvers.solverslib.command.WaitCommand;
//import org.firstinspires.ftc.teamcode.SubSystems.Juggler;
//
//public class JiggleCommand extends SequentialCommandGroup {
//
//    public JiggleCommand(Juggler juggler) {
//        addRequirements(juggler);
//
//        addCommands(
//                new InstantCommand(() -> juggler.startSlowSpin(Juggler.Direction.CW)), // small CW spin
//                new WaitCommand(200), // wait for sensor to read
//                new InstantCommand(() -> juggler.startSlowSpin(Juggler.Direction.CCW)), // spin back
//                new WaitCommand(200),
//                new InstantCommand(juggler::Snap) // snap to nearest slot
//        );
//    }
//}
