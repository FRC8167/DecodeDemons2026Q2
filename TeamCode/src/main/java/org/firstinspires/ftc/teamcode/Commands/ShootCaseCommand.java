package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.SubSystems.ColorMatch;
import org.firstinspires.ftc.teamcode.SubSystems.Juggler;
import org.firstinspires.ftc.teamcode.SubSystems.Popper;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter;
import org.firstinspires.ftc.teamcode.SubSystems.Vision;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class ShootCaseCommand extends CommandBase {

    private final Juggler juggler;
    private final Popper popper;
    private final Shooter shooter;
    private final ColorMatch colorMatch;
    private final Vision vision;
    private SequentialCommandGroup sequence;
    //create a list from the color array of available options
    private final List<ColorMatch.ArtifactColor> remainingMotif = new ArrayList<>();
    private int jiggleAttempts = 0;
    private final int maxJiggles = 2;

    private boolean isJiggling = false;
    private final ElapsedTime jiggleTimer = new ElapsedTime();

    private boolean kicking = false;
    private final ElapsedTime kickTimer = new ElapsedTime();

//    private boolean jugglerRotating = false;

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
    }

    @Override
    public void initialize() {
        // latch the motif from vision
        ColorMatch.ArtifactColor[] motif = vision.getLatchedMotif();
        if (motif != null && motif.length > 0) {
            remainingMotif.addAll(Arrays.asList(motif));
        }
    }

    @Override
    public void execute() {
        if (remainingMotif.isEmpty()) return;

        ColorMatch.ArtifactColor target = remainingMotif.get(0);

        ColorMatch.ArtifactColor s0 = colorMatch.detectColor(ColorMatch.Slot.SLOT_0);
        ColorMatch.ArtifactColor s1 = colorMatch.detectColor(ColorMatch.Slot.SLOT_1);
        ColorMatch.ArtifactColor s2 = colorMatch.detectColor(ColorMatch.Slot.SLOT_2);
        boolean targetVisible = (s0 == target || s1 == target || s2 == target);


        //if target motif color in slot0, shoot!!!


        if (s0 == target) {
            shooter.smartVelocity(vision.getDistanceToGoal(), s0);
            if (shooter.atTargetVelocity()) {
                if (!kicking) {
                    popper.set(Popper.PopperState.KICK);
                    kickTimer.reset();
                    kicking = true;
                }
                if (kicking && kickTimer.milliseconds() > 250) {
                    popper.set(Popper.PopperState.RESET);
                    kicking = false;
                }

                remainingMotif.remove(0);
//                jugglerRotating = false;
                jiggleAttempts = 0;
            }
        //if target motif color in slot1, rotate 1 slot CW and then shoot!
        } else if (s1 == target) {
            new RotateOneSlotCommand(juggler, Juggler.Direction.CW).schedule();

        //if target motif color in slot2, rotate 1 slot SSW and then shoot!
        } else if (s2 == target) {
            new RotateOneSlotCommand(juggler, Juggler.Direction.CCW).schedule();
//            juggler.rotateOneSlot(Juggler.Direction.CCW);  //old way

        //if target not found in any slot, jiggle up to two times.


        } else if (!targetVisible && jiggleAttempts < maxJiggles && !isJiggling) {

            // start jiggle one time
            juggler.startSlowSpin(Juggler.Direction.CW);
            jiggleTimer.reset();

            isJiggling = true;

        } else {
            // FALLBACK: shoot whatever is in slot0
            shooter.smartVelocity(vision.getDistanceToGoal(), s0);

            if (shooter.atTargetVelocity()) {
                if (!kicking) {
                    popper.set(Popper.PopperState.KICK);
                    kickTimer.reset();
                    kicking = true;
                }
                if (kicking && kickTimer.milliseconds() >= 250) {
                    popper.set(Popper.PopperState.RESET);
                    kicking = false;

                    remainingMotif.remove(0);   // move to next artifact
//                    jugglerRotating = false;
                    jiggleAttempts = 0;
                }
            }
        }

        if (isJiggling && jiggleTimer.milliseconds() >= 250) {
            juggler.Snap();
            jiggleAttempts++;
            isJiggling = false;
        }

    }


    @Override
    public boolean isFinished() {
        return remainingMotif.isEmpty();  //when no artifacts left to shoot
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }

//    @Override
//    public void initialize() {
//        super.initialize();
//        // Attempt to read the motif
//        ColorMatch.ArtifactColor[] motif = vision.getLatchedMotif();
////        assert motif != null;  //this was causing the crash!
//
////        // If motif is missing, schedule a jiggle to help the sensor
////        if (motif == null || motif.length < 3) {
////
////            // Retry reading after jiggle
////            motif = vision.getLatchedMotif();
////        }
////
////
////        int attempts = 0;
//        // Build case key safely
//        String caseKey = buildCaseKey(motif, colorMatch);
//
////        while (attempts < 2 && caseKey.contains("U")) {
////
////                caseKey = "UNKNOWN";
////                new JiggleCommand(juggler).schedule();
////                attempts += 1;
////            }}
//        sequence = buildSequence(caseKey);
//        if (sequence == null) {
//            sequence = new SequentialCommandGroup();
//        }
//        sequence.initialize();
////        throw new RuntimeException("Motif: " + Arrays.toString(motif) +"\n"+"Key: "+caseKey);
//
//    }
//
//    @Override
//    public void execute() {
//        sequence.execute();
//    }
//
//    @Override
//    public boolean isFinished() {
//        return sequence.isFinished();
//    }
//
//    private void SeqCmdAuto(String key, ColorMatch colorMatch){
//
//       colorMatch.detectColor(ColorMatch.Slot.SLOT_0);
//
//    }
//    private SequentialCommandGroup buildSequence(String key) {
//
//        if (colorMatch.detectColor(ColorMatch.Slot.SLOT_0) == ColorMatch.ArtifactColor.UNKNOWN)
//            new RotateXSlotsCommand(juggler, Juggler.Direction.CCW, 1);
//
//
//        switch (key) {
//
//            case "MPGPJGPP":
//                if (colorMatch.detectColor(ColorMatch.Slot.SLOT_0) == ColorMatch.ArtifactColor.UNKNOWN)
//                    new RotateXSlotsCommand(juggler, Juggler.Direction.CCW, 1);
//
//                return new SequentialCommandGroup(
//                        new ParallelCommandGroup(
//                            new ShooterSmartSpinUpCommand(shooter, vision),
//                            new RotateXSlotsCommand(juggler, Juggler.Direction.CCW, 1)
//                        ),
//                        new PopandResetCommand(popper),
//                        new ParallelCommandGroup(
//                            new ShooterSmartSpinUpCommand(shooter, vision),
//                            new RotateXSlotsCommand(juggler, Juggler.Direction.CW, 1)
//                        ),
//                        new PopandResetCommand(popper),
//                        new ParallelCommandGroup(
//                                new ShooterSmartSpinUpCommand(shooter, vision),
//                            new RotateXSlotsCommand(juggler, Juggler.Direction.CW, 1)
//                        ),
//                        new PopandResetCommand(popper),
//                        new InstantCommand(shooter::stop)
//                );
//
//            case "MPPGJGPP":
//            case "MGPPJPPG":
//                if (colorMatch.detectColor(ColorMatch.Slot.SLOT_0) == ColorMatch.ArtifactColor.UNKNOWN)
//                    new RotateXSlotsCommand(juggler, Juggler.Direction.CCW, 1);
//
//                return new SequentialCommandGroup(
//                        new ParallelCommandGroup(
//                            new ShooterSmartSpinUpCommand(shooter, vision),
//                            new RotateXSlotsCommand(juggler, Juggler.Direction.CCW, 1)
//                        ),
//                        new PopandResetCommand(popper),
//                        new ParallelCommandGroup(
//                             new ShooterSmartSpinUpCommand(shooter, vision),
//                             new RotateXSlotsCommand(juggler, Juggler.Direction.CCW, 1)
//                        ),
//                        new PopandResetCommand(popper),
//                        new ParallelCommandGroup(
//                            new ShooterSmartSpinUpCommand(shooter, vision),
//                            new RotateXSlotsCommand(juggler, Juggler.Direction.CCW, 1)
//                        ),
//                        new PopandResetCommand(popper),
//                        new InstantCommand(shooter::stop)
//                );
//
//            case "MPPGJPGP":
//            case "MPGPJPPG":
//                if (colorMatch.detectColor(ColorMatch.Slot.SLOT_0) == ColorMatch.ArtifactColor.UNKNOWN)
//                    new RotateXSlotsCommand(juggler, Juggler.Direction.CCW, 1);
//
//                return new SequentialCommandGroup(
//                        new ShooterSmartSpinUpCommand(shooter, vision),
//                        new PopandResetCommand(popper),
//                        new ParallelCommandGroup(
//                            new ShooterSmartSpinUpCommand(shooter, vision),
//                            new RotateXSlotsCommand(juggler, Juggler.Direction.CCW, 1)
//                        ),
//                        new PopandResetCommand(popper),
//                        new ParallelCommandGroup(
//                            new ShooterSmartSpinUpCommand(shooter, vision),
//                            new RotateXSlotsCommand(juggler, Juggler.Direction.CCW, 1)
//                        ),
//                        new PopandResetCommand(popper),
//                        new InstantCommand(shooter::stop)
//                );
//
//            case "MGPPJPGP":
//                if (colorMatch.detectColor(ColorMatch.Slot.SLOT_0) == ColorMatch.ArtifactColor.UNKNOWN)
//                    new RotateXSlotsCommand(juggler, Juggler.Direction.CCW, 1);
//
//                return new SequentialCommandGroup(
//                        new ParallelCommandGroup(
//                            new ShooterSmartSpinUpCommand(shooter, vision),
//                            new RotateXSlotsCommand(juggler, Juggler.Direction.CW, 1)
//                        ),
//                        new PopandResetCommand(popper),
//                        new ParallelCommandGroup(
//                            new ShooterSmartSpinUpCommand(shooter, vision),
//                            new RotateXSlotsCommand(juggler, Juggler.Direction.CW, 1)
//                        ),
//                        new PopandResetCommand(popper),
//                        new ParallelCommandGroup(
//                            new ShooterSmartSpinUpCommand(shooter, vision),
//                            new RotateXSlotsCommand(juggler, Juggler.Direction.CW, 1)
//                        ),
//                        new PopandResetCommand(popper),
//                        new InstantCommand(shooter::stop)
//                );
//
//            case "MPPGJPPG":
//            case "MPGPJPGP":
//            case "MGPPJGPP":
//            case "MPPGJGGG":
//            case "MPGPJGGG":
//            case "MGPPJGGG":
//            case "UNKNOWN":
//            default:
//                if (colorMatch.detectColor(ColorMatch.Slot.SLOT_0) == ColorMatch.ArtifactColor.UNKNOWN)
//                    new RotateXSlotsCommand(juggler, Juggler.Direction.CCW, 1);
//                return new SequentialCommandGroup(
//                        new ShooterSmartSpinUpCommand(shooter, vision),
//                        new PopandResetCommand(popper),
//                        new ParallelCommandGroup(
//                            new ShooterSmartSpinUpCommand(shooter, vision),
//                            new RotateXSlotsCommand(juggler, Juggler.Direction.CW, 1)
//                        ),
////                        new WaitCommand(500),
//                        new PopandResetCommand(popper),
//                        new ParallelCommandGroup(
//                                new ShooterSmartSpinUpCommand(shooter, vision),
//                                new RotateXSlotsCommand(juggler, Juggler.Direction.CW, 1)
//                        ),
//                        new PopandResetCommand(popper)
////                        new InstantCommand(shooter::stop)
//                );
//        }
//    }
//
//    private String buildCaseKey(
//            ColorMatch.ArtifactColor[] motif,
//            ColorMatch colorMatch
//
//    ) {
//
//        char Slot0_char = colorToChar(colorMatch.detectColor(ColorMatch.Slot.SLOT_0));
//        char Slot1_char = colorToChar(colorMatch.detectColor(ColorMatch.Slot.SLOT_1));
//        char Slot2_char = colorToChar(colorMatch.detectColor(ColorMatch.Slot.SLOT_2));
//        if (motif == null || motif.length < 3 || Slot0_char == 'U' || Slot1_char == 'U' || Slot2_char == 'U') {
//
//            return "UNKNOWN";
//        }
//
//        StringBuilder sb = new StringBuilder("M");
//        for (ColorMatch.ArtifactColor c : motif) {
//            sb.append(colorToChar(c));
//        }
//
//        sb.append("J");
//        sb.append(Slot0_char);
//        sb.append(Slot1_char);
//        sb.append(Slot2_char);
//
//        return sb.toString();
//    }
//
//    private char colorToChar(ColorMatch.ArtifactColor c) {
//        switch (c) {
//            case PURPLE: return 'P';
//            case GREEN:  return 'G';
//            default:     return 'U';
//        }
//    }
//

}
