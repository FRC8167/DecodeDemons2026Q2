package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.SubSystems.ColorMatch;
import org.firstinspires.ftc.teamcode.SubSystems.Juggler;
import org.firstinspires.ftc.teamcode.SubSystems.Popper;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter;
import org.firstinspires.ftc.teamcode.SubSystems.Vision;

public class ShootCaseCommand extends CommandBase {

    private final Juggler juggler;
    private final Popper popper;
    private final Shooter shooter;
    private final ColorMatch colorMatch;
    private final Vision vision;

    private String caseKey;
    private int step;
    private boolean rotating = false;
    private boolean popping = false;
    private boolean spinningUp = false;

    private final ElapsedTime timer = new ElapsedTime();
    private static final long POP_TIME_MS = 500;
    private static final long SPINUP_TIME_MS = 400; // time to reach target RPM

    public ShootCaseCommand(
            Juggler juggler,
            Popper popper,
            Shooter shooter,
            ColorMatch colorMatch,
            Vision vision
    ) {
        this.juggler = juggler;
        this.popper = popper;
        this.shooter = shooter;
        this.colorMatch = colorMatch;
        this.vision = vision;

        addRequirements(juggler, popper);
    }

    @Override
    public void initialize() {
        ColorMatch.ArtifactColor[] motif = vision.getLatchedMotif();
        caseKey = buildCaseKey(motif, colorMatch);

        if (caseKey.contains("U")) {
            caseKey = "UNKNOWN";
        }

        step = 0;
        rotating = false;
        popping = false;

        popper.set(Popper.PopperState.RESET);
        timer.reset();
    }

    @Override
    public void execute() {

        switch (caseKey) {
            //stack cases when branches are duplicates
            case "MPGPJPGP":
            case "MPPGJPPG":
            case "MGPPJGPP":
                // motif and slots already match
                runSteps(
                        () -> justPop(),  // first ball
                        () -> rotateThenPop(Juggler.Direction.CW),  // second ball
                        () -> rotateThenPop(Juggler.Direction.CW)  // third ball
                );
                break;

            case "MPGPJPPG":
            case "MPPGJPGP":
                runSteps(
                        () -> justPop(),
                        () -> rotateThenPop(Juggler.Direction.CCW),
                        () -> rotateThenPop(Juggler.Direction.CCW)
                );
                break;

            case "MPGPJGPP":
                runSteps(
                        () -> rotateThenPop(Juggler.Direction.CCW),
                        () -> rotateThenPop(Juggler.Direction.CW),
                        () -> rotateThenPop(Juggler.Direction.CW)
                );
                break;

            case "MPPGJGPP":
            case "MGPPJPGP":
                new SequentialCommandGroup(
                        new RotateXSlotsCommand(juggler, Juggler.Direction.CW, 1),
                        new ShooterSmartSpinUpCommand(shooter, vision),
                        new PopandResetCommand(popper),
                        new RotateXSlotsCommand(juggler, Juggler.Direction.CW, 1),
                        new ShooterSmartSpinUpCommand(shooter, vision),
                        new PopandResetCommand(popper),
                        new RotateXSlotsCommand(juggler, Juggler.Direction.CW, 1),
                        new ShooterSmartSpinUpCommand(shooter, vision),
                        new PopandResetCommand(popper),
                        new ShooterSpinUpCommand(shooter,
                                0.0)
                );
                break;

            case "MGPPJPPG":
            case "UNKNOWN":
                runSteps(
                        () -> rotateThenPop(Juggler.Direction.CW),
                        () -> rotateThenPop(Juggler.Direction.CW),
                        () -> rotateThenPop(Juggler.Direction.CW)
                );
                break;
        }
    }

    private void runSteps(Runnable s0, Runnable s1, Runnable s2) {
        if (step == 0) s0.run();
        else if (step == 1) s1.run();
        else if (step == 2) s2.run();
    }


    private void justPop() {

        //Spin up shooter
        if (!spinningUp) {
            shooter.setVelocity(3850);  // start shooter motor
            spinningUp = true;
            return;
        }
        // Wait until shooter at target velocity
        if (!shooter.atTargetVelocity()) return;

        // Cycle the poppers
        if (!popping) {
            popper.set(Popper.PopperState.KICK);
            timer.reset();
            popping = true;
            return;
        }
        //Wait a 1/2 second and then reset poppers
        if (timer.milliseconds() < POP_TIME_MS) return;
        //Reset poppers and stop shooter
        popper.set(Popper.PopperState.RESET);
        shooter.stop();
        //shooter.setVelocity(0);
        //Finish the cycle
        spinningUp = false;
        popping = false;
        step+=1;
    }


    private void rotateThenPop(Juggler.Direction dir) {
        if (!rotating) {
            juggler.rotateOneSlot(dir);
            rotating = true;
//            return;
        }
        if (juggler.atTarget()) rotating = false;

        //Spin up shooter
        if (!spinningUp) {
            shooter.setVelocity(3850);  // start shooter motor
            spinningUp = true;
        }
        // Wait until shooter at target velocity
//        if (!shooter.atTargetVelocity()) return;

        // Cycle the poppers
        if(!popping){
//        if (shooter.atTargetVelocity() && !popping) {
            popper.set(Popper.PopperState.KICK);
            timer.reset();
            popping = true;
        }
        //Wait a 1/2 second and then reset poppers
        if (timer.milliseconds() < POP_TIME_MS) return;

        //Reset poppers and stop shooter
        popper.set(Popper.PopperState.RESET);
        shooter.stop();
        //shooter.setVelocity(0);
        //Finish the cycle
        spinningUp = false;
        popping = false;
        step+=1;
    }


    @Override
    public boolean isFinished() {
        return step >= 3;
    }

    private String buildCaseKey(
            ColorMatch.ArtifactColor[] motif,
            ColorMatch colorMatch
    ) {
        StringBuilder sb = new StringBuilder("M");

        for (ColorMatch.ArtifactColor c : motif) {
            sb.append(colorToChar(c));
        }

        sb.append("J");
        sb.append(colorToChar(colorMatch.detectColor(ColorMatch.Slot.SLOT_0)));
        sb.append(colorToChar(colorMatch.detectColor(ColorMatch.Slot.SLOT_1)));
        sb.append(colorToChar(colorMatch.detectColor(ColorMatch.Slot.SLOT_2)));

        return sb.toString();
    }

    private char colorToChar(ColorMatch.ArtifactColor c) {
        switch (c) {
            case PURPLE: return 'P';
            case GREEN:  return 'G';
            default:     return 'U';
        }
    }
}
