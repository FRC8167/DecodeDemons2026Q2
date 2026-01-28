package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

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

    private final List<ColorMatch.ArtifactColor> remainingMotif = new ArrayList<>();
    private int jiggleAttempts = 0;
    private final int maxJiggles = 2;

    private boolean isJiggling = false;
    private final ElapsedTime jiggleTimer = new ElapsedTime();

    private boolean kicking = false;
    private final ElapsedTime kickTimer = new ElapsedTime();

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
        // Reset for reuse
        remainingMotif.clear();
        jiggleAttempts = 0;
        isJiggling = false;
        kicking = false;

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

        // Update virtual slots each loop
        ColorMatch.ArtifactColor s0 = colorMatch.detectColor(ColorMatch.Slot.SLOT_0); // under shooter
        ColorMatch.ArtifactColor s1 = colorMatch.detectColor(ColorMatch.Slot.SLOT_1); // forward-right
        ColorMatch.ArtifactColor s2 = colorMatch.detectColor(ColorMatch.Slot.SLOT_2); // forward-left

        // Slot 0 under shooter, Slot 1 CW to Slot 0, Slot 2 CCW to Slot 0
        if (s0 == target) {
            shooter.smartVelocity(vision.getDistanceToGoal(), s0);
            if (shooter.atTargetVelocity()) {
                if (!kicking) {
                    popper.set(Popper.PopperState.KICK);
                    kickTimer.reset();
                    kicking = true;
                } else if (kickTimer.milliseconds() > 250) {
                    popper.set(Popper.PopperState.RESET);
                    kicking = false;
                }

                remainingMotif.remove(0);
                jiggleAttempts = 0;
            }

        } else if (s1 == target) {
            // CW rotation moves slot1 -> slot0
            new RotateOneSlotCommand(juggler, Juggler.Direction.CW).schedule();

        } else if (s2 == target) {
            // CCW rotation moves slot2 -> slot0
            new RotateOneSlotCommand(juggler, Juggler.Direction.CCW).schedule();

        } else if (jiggleAttempts < maxJiggles && !isJiggling) {
            // start jiggle one time
            juggler.startSlowSpin(Juggler.Direction.CW);
            jiggleTimer.reset();
            isJiggling = true;

        } else if (!isJiggling) {
            // FALLBACK: shoot whatever is in slot0
            shooter.smartVelocity(vision.getDistanceToGoal(), s0);
            if (shooter.atTargetVelocity()) {
                if (!kicking) {
                    popper.set(Popper.PopperState.KICK);
                    kickTimer.reset();
                    kicking = true;
                } else if (kickTimer.milliseconds() >= 250) {
                    popper.set(Popper.PopperState.RESET);
                    kicking = false;

                    remainingMotif.remove(0);
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
        return remainingMotif.isEmpty();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }
}








//package org.firstinspires.ftc.teamcode.Commands;
//
//import com.seattlesolvers.solverslib.command.CommandBase;
//import com.seattlesolvers.solverslib.command.InstantCommand;
//import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
//import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
//
//import org.firstinspires.ftc.teamcode.SubSystems.ColorMatch;
//import org.firstinspires.ftc.teamcode.SubSystems.Juggler;
//import org.firstinspires.ftc.teamcode.SubSystems.Popper;
//import org.firstinspires.ftc.teamcode.SubSystems.Shooter;
//import org.firstinspires.ftc.teamcode.SubSystems.Vision;
//
//public class ShootCaseCommand extends CommandBase {
//
//    private final Juggler juggler;
//    private final Popper popper;
//    private final Shooter shooter;
//    private final ColorMatch colorMatch;
//    private final Vision vision;
//    // private SequentialCommandGroup sequence;
//    // private SeqCmdAuto sequence;
//    private SequentialCommandGroup sequence;
//
//    public ShootCaseCommand(
//            Juggler juggler,
//            Popper popper,
//            Shooter shooter,
//            ColorMatch colorMatch,
//            Vision vision) {
//        this.juggler = juggler;
//        this.popper = popper;
//        this.shooter = shooter;
//        this.colorMatch = colorMatch;
//        this.vision = vision;
//    }
//
//    @Override
//    public void initialize() {
//        super.initialize();
//        // Attempt to read the motif
//        ColorMatch.ArtifactColor[] motif = vision.getLatchedMotif();
//
//        sequence = buildAutoSequence(motif, colorMatch);
//        if (sequence == null) {
//            sequence = new SequentialCommandGroup();
//            // sequence = new SeqCmdAuto(caseKey, colorMatch);
//        }
//        sequence.initialize();
//        // throw new RuntimeException("Motif: " + Arrays.toString(motif) +"\n"+"Key:
//        // "+caseKey);
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
//    private SequentialCommandGroup buildAutoSequence(ColorMatch.ArtifactColor[] motif, ColorMatch colorMatch) {
//        if (motif == null || motif.length != 3) {
//            // TODO: Handle this case
//            // Set random motif
//            motif = new ColorMatch.ArtifactColor[] {
//                    ColorMatch.ArtifactColor.PURPLE,
//                    ColorMatch.ArtifactColor.PURPLE,
//                    ColorMatch.ArtifactColor.GREEN
//            };
//        }
//
//        SequentialCommandGroup seq = new SequentialCommandGroup();
//
//        // Track virtual slots: [Slot 0, Slot 1, Slot 2]
//        ColorMatch.ArtifactColor[] virtualSlots = new ColorMatch.ArtifactColor[3];
//        virtualSlots[0] = colorMatch.detectColor(ColorMatch.Slot.SLOT_0);
//        virtualSlots[1] = colorMatch.detectColor(ColorMatch.Slot.SLOT_1);
//        virtualSlots[2] = colorMatch.detectColor(ColorMatch.Slot.SLOT_2);
//
//        // Count total artifacts initially
//        int totalArtifacts = 0;
//        for (ColorMatch.ArtifactColor c : virtualSlots) {
//            if (c != ColorMatch.ArtifactColor.NONE) {
//                totalArtifacts++;
//            }
//        }
//
//        // If Slot 0 is unknown/empty initially, maybe rotate to fill it?
//        // Logic below handles selection. If Slot 0 is UNKNOWN but needed, it might be
//        // an issue,
//        // but let's assume detection works.
//        // Existing code had a check:
//        // if (colorMatch.detectColor(ColorMatch.Slot.SLOT_0) ==
//        // ColorMatch.ArtifactColor.UNKNOWN)
//        // new RotateXSlotsCommand(juggler, Juggler.Direction.CCW, 1);
//        // This implies if S0 is empty, we rotate CCW 1 (Move S1->S0).
//        // I will replicate this behavior as a pre-step if S0 is empty,
//        // but only if we don't handle it in the loop logic.
//        // Actually, let's trust the loop logic. If we need a color and it's at S1, we
//        // will rotate CCW 1 regardless.
//
//        for (ColorMatch.ArtifactColor targetColor : motif) {
//            // Stop if we ran out of artifacts
//            if (totalArtifacts <= 0) {
//                break;
//            }
//
//            // Find index of targetColor in virtualSlots
//            int targetIndex = -1;
//
//            // Prefer Slot 0, then 1, then 2 (or based on rotation cost? 0 is best).
//            if (virtualSlots[0] == targetColor)
//                targetIndex = 0;
//            else if (virtualSlots[1] == targetColor)
//                targetIndex = 1;
//            else if (virtualSlots[2] == targetColor)
//                targetIndex = 2;
//
//            if (targetIndex == -1) {
//                // Color not found in slots!
//                // Maybe it's UNKNOWN? Or we just proceed/skip?
//                // For safety, let's just log or break.
//                // But we must return something valid.
//                continue;
//            }
//
//            CommandBase action = null;
//
//            if (targetIndex == 0) {
//                // Already at 0. Just shoot.
//                action = new ShooterSmartSpinUpCommand(shooter, vision);
//            } else if (targetIndex == 1) {
//                // At Slot 1. Rotate CCW 1 to bring S1 -> S0.
//                action = new ParallelCommandGroup(
//                        new ShooterSmartSpinUpCommand(shooter, vision),
//                        new RotateXSlotsCommand(juggler, Juggler.Direction.CCW, 1));
//
//                // Update virtual slots: CCW 1
//                // S0 <- S1
//                // S1 <- S2
//                // S2 <- S0 (old S0)
//                ColorMatch.ArtifactColor temp = virtualSlots[0];
//                virtualSlots[0] = virtualSlots[1];
//                virtualSlots[1] = virtualSlots[2];
//                virtualSlots[2] = temp;
//
//            } else if (targetIndex == 2) {
//                // At Slot 2. Rotate CW 1 to bring S2 -> S0.
//                action = new ParallelCommandGroup(
//                        new ShooterSmartSpinUpCommand(shooter, vision),
//                        new RotateXSlotsCommand(juggler, Juggler.Direction.CW, 1));
//
//                // Update virtual slots: CW 1
//                // S0 <- S2
//                // S2 <- S1
//                // S1 <- S0 (old S0)
//                ColorMatch.ArtifactColor temp = virtualSlots[0];
//                virtualSlots[0] = virtualSlots[2];
//                virtualSlots[2] = virtualSlots[1];
//                virtualSlots[1] = temp;
//            }
//
//            seq.addCommands(action);
//            seq.addCommands(new PopandResetCommand(popper));
//
//            // Artifact at 0 is now used/popped.
//            virtualSlots[0] = ColorMatch.ArtifactColor.NONE;
//            totalArtifacts--;
//
//            // Stop shooter at the very end?
//            // The original code has `new InstantCommand(shooter::stop))` at the end of the
//            // sequence.
//        }
//
//        seq.addCommands(new InstantCommand(shooter::stop));
//        return seq;
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
//            case PURPLE:
//                return 'P';
//            case GREEN:
//                return 'G';
//            default:
//                return 'U';
//        }
//    }
//
//    // Are there 3 artifacts identified?
//    private boolean hasThreeArtifacts(ColorMatch colorMatch) {
//
//        char Slot0_char = colorToChar(colorMatch.detectColor(ColorMatch.Slot.SLOT_0));
//        char Slot1_char = colorToChar(colorMatch.detectColor(ColorMatch.Slot.SLOT_1));
//        char Slot2_char = colorToChar(colorMatch.detectColor(ColorMatch.Slot.SLOT_2));
//
//        if ((Slot0_char == 'P' || Slot0_char == 'G') &&
//                (Slot1_char == 'P' || Slot1_char == 'G') &&
//                (Slot2_char == 'P' || Slot2_char == 'G'))
//            return true;
//
//        return false;
//
//    }
//
//}

//DMW's CODE
//package org.firstinspires.ftc.teamcode.Commands;
//
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.seattlesolvers.solverslib.command.CommandBase;
//import com.seattlesolvers.solverslib.command.InstantCommand;
//import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
//import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
//import com.seattlesolvers.solverslib.command.WaitCommand;
//
//import org.firstinspires.ftc.teamcode.SubSystems.ColorMatch;
//import org.firstinspires.ftc.teamcode.SubSystems.Juggler;
//import org.firstinspires.ftc.teamcode.SubSystems.Popper;
//import org.firstinspires.ftc.teamcode.SubSystems.Shooter;
//import org.firstinspires.ftc.teamcode.SubSystems.Vision;
//
//import java.util.ArrayList;
//import java.util.Arrays;
//import java.util.List;
//
//public class ShootCaseCommand extends CommandBase {
//
//    private final Juggler juggler;
//    private final Popper popper;
//    private final Shooter shooter;
//    private final ColorMatch colorMatch;
//    private final Vision vision;
//    private SequentialCommandGroup sequence;
//    //create a list from the color array of available options
//    private final List<ColorMatch.ArtifactColor> remainingMotif = new ArrayList<>();
//    private int jiggleAttempts = 0;
//    private final int maxJiggles = 2;
//
//    private boolean isJiggling = false;
//    private final ElapsedTime jiggleTimer = new ElapsedTime();
//
//    private boolean kicking = false;
//    private final ElapsedTime kickTimer = new ElapsedTime();
//
////    private boolean jugglerRotating = false;
//
//    public ShootCaseCommand(
//            Juggler juggler,
//            Popper popper,
//            Shooter shooter,
//            ColorMatch colorMatch,
//            Vision vision
//    ) {
//        this.juggler = juggler;
//        this.popper = popper;
//        this.shooter = shooter;
//        this.colorMatch = colorMatch;
//        this.vision = vision;
//    }
//
//    @Override
//    public void initialize() {
//        // latch the motif from vision
//        ColorMatch.ArtifactColor[] motif = vision.getLatchedMotif();
//        if (motif != null && motif.length > 0) {
//            remainingMotif.addAll(Arrays.asList(motif));
//        }
//    }
//
//    @Override
//    public void execute() {
//        if (remainingMotif.isEmpty()) return;
//
//        ColorMatch.ArtifactColor target = remainingMotif.get(0);
//
//        ColorMatch.ArtifactColor s0 = colorMatch.detectColor(ColorMatch.Slot.SLOT_0);
//        ColorMatch.ArtifactColor s1 = colorMatch.detectColor(ColorMatch.Slot.SLOT_1);
//        ColorMatch.ArtifactColor s2 = colorMatch.detectColor(ColorMatch.Slot.SLOT_2);
//        boolean targetVisible = (s0 == target || s1 == target || s2 == target);
//
//
//        //if target motif color in slot0, shoot!!!
//
//
//        if (s0 == target) {
//            shooter.smartVelocity(vision.getDistanceToGoal(), s0);
//            if (shooter.atTargetVelocity()) {
//                if (!kicking) {
//                    popper.set(Popper.PopperState.KICK);
//                    kickTimer.reset();
//                    kicking = true;
//                } else if (kickTimer.milliseconds() > 250) {
//                    popper.set(Popper.PopperState.RESET);
//                    kicking = false;
//                }
//
//                remainingMotif.remove(0);
////                jugglerRotating = false;
//                jiggleAttempts = 0;
//            }
//        //if target motif color in slot1, rotate 1 slot CW and then shoot!
//        } else if (s1 == target) {
//            new RotateOneSlotCommand(juggler, Juggler.Direction.CW).schedule();
//
//        //if target motif color in slot2, rotate 1 slot SSW and then shoot!
//        } else if (s2 == target) {
//            new RotateOneSlotCommand(juggler, Juggler.Direction.CCW).schedule();
////            juggler.rotateOneSlot(Juggler.Direction.CCW);  //old way
//
//        //if target not found in any slot, jiggle up to two times.
//
//
//        } else if (jiggleAttempts < maxJiggles && !isJiggling) {
//
//            // start jiggle one time
//            juggler.startSlowSpin(Juggler.Direction.CW);
//            jiggleTimer.reset();
//
//            isJiggling = true;
//
//        } else if (!isJiggling) {
//            // FALLBACK: shoot whatever is in slot0
//            shooter.smartVelocity(vision.getDistanceToGoal(), s0);
//
//            if (shooter.atTargetVelocity()) {
//                if (!kicking) {
//                    popper.set(Popper.PopperState.KICK);
//                    kickTimer.reset();
//                    kicking = true;
//                } else if (kickTimer.milliseconds() >= 250) {
//                    popper.set(Popper.PopperState.RESET);
//                    kicking = false;
//
//                    remainingMotif.remove(0);   // move to next artifact
////                    jugglerRotating = false;
//                    jiggleAttempts = 0;
//                }
//            }
//        }
//
//        if (isJiggling && jiggleTimer.milliseconds() >= 250) {
//            juggler.Snap();
//            jiggleAttempts++;
//            isJiggling = false;
//        }
//
//    }
//
//
//    @Override
//    public boolean isFinished() {
//        return remainingMotif.isEmpty();  //when no artifacts left to shoot
//    }
//
//    @Override
//    public void end(boolean interrupted) {
//        shooter.stop();
//    }



//}
