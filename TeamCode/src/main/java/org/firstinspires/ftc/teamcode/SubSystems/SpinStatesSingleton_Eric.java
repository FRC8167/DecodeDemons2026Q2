package org.firstinspires.ftc.teamcode.SubSystems;


import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.Cogintilities.State;
import org.firstinspires.ftc.teamcode.Cogintilities.TeamConstants;
import org.jetbrains.annotations.Contract;

import java.util.Arrays;

public class SpinStatesSingleton_Eric implements TeamConstants {
    private volatile State slot0;
    private volatile State slot1;
    private volatile State slot2;
    //Note: these slots are different from their original position on Devils.
    //slot0 is positioned under the shooter with increasing ccw to correlate with sensors at initial position
    //Note2: rotating juggler to index 1 results in a cw movement of the spindexer rotating the positions to now be 2,0,1 ccw
    //In other words index 0 is slot 0, index 1 is slot 2, index 2 is slot 1 (annoying, I know)

    private static volatile SpinStatesSingleton_Eric single_instance = null;

    private SpinStatesSingleton_Eric() {
        slot0 = State.NONE;
        slot1 = State.NONE;
        slot2 = State.NONE;
    }

    public static synchronized SpinStatesSingleton_Eric getInstance()
    {
        if (single_instance == null)
            single_instance = new SpinStatesSingleton_Eric();
        return single_instance;
    }

    public synchronized void setSlot(int index, State state) {
        if (index == 0) {
            slot0 = state;
        } else if (index == 1) {
            slot1 = state;
        } else if (index == 2) {
            slot2 = state;
        } else {
            throw new IllegalArgumentException("Index must be between 0-2");
        }
    }

    public synchronized State getSlot(int index) {
        if (index == 0) {
            return slot0;
        } else if (index == 1) {
            return slot1;
        } else if (index == 2) {
            return slot2;
        } else {
            throw new IllegalArgumentException("Index must be between 0-2");
        }
    }
    //Note: Method creates garbage and shouldn't be used unless necessary
//    public synchronized State[] getStates() {
//        return new State[] {slot0, slot1, slot2};
//    }

    public synchronized int getCountOfStateInStates(State state, int... excludedIndexes) {
        int count =0;
        if (slot0 == state && isNotExcluded(0, excludedIndexes)) count++;
        if (slot1 == state && isNotExcluded(1, excludedIndexes)) count++;
        if (slot2 == state && isNotExcluded(2, excludedIndexes)) count++;
        return count;
    }

    public synchronized boolean isStateInStates(State state, int... excludedIndexes) {
        return
                slot0 == state && isNotExcluded(0, excludedIndexes) ||
                slot1 == state && isNotExcluded(1, excludedIndexes) ||
                slot2 == state && isNotExcluded(2, excludedIndexes);
    }

    public synchronized int[] getIndexesOfStateInStates(State state, int... excludedIndexes) {
        // 1. Calculate count using fields directly (No new State[] allocation)
        int count = 0;
        if (slot0 == state && isNotExcluded(0, excludedIndexes)) count++;
        if (slot1 == state && isNotExcluded(1, excludedIndexes)) count++;
        if (slot2 == state && isNotExcluded(2, excludedIndexes)) count++;

        if (count == 0) return new int[0];

        // 2. The data cannot change here because we hold the 'synchronized' lock
        int[] indexes = new int[count];
        int currentIndex = 0;

        if (slot0 == state && isNotExcluded(0, excludedIndexes)) indexes[currentIndex++] = 0;
        if (slot1 == state && isNotExcluded(1, excludedIndexes)) indexes[currentIndex++] = 1;
        if (slot2 == state && isNotExcluded(2, excludedIndexes)) indexes[currentIndex++] = 2;

        return indexes;
    }

    @Contract(pure = true)
    public static boolean isNotExcluded(int index, @NonNull int... excludedIndexes) {
        for (int ex : excludedIndexes) {
            if (ex == index) return false;
        }
        return true;
    }

//    public String convertStatesToInitials(State... states) {
//        if (states == null) {
//            return "";
//        } else {
//            StringBuilder string = new StringBuilder();
//            for (State state : states) {
//                string.append(state.getCharacter());
//            }
//            return string.toString();
//        }
//    }

    public static State[] getNextToShoot(int scored, State[] sequence) {
        if (sequence == null)
            return null;
        int index1 = scored % 3;
        int index2 = (scored+1) % 3;
        int index3 = (scored+2) % 3;
        return new State[]{sequence[index1], sequence[index2], sequence[index3]};
    }


    public static State get1stNextToShoot(int scored, State[] sequence) {
        if (sequence == null)
            return State.NONE;
        int index = scored % 3;
        return sequence[index];
    }

    public static State get2ndNextToShoot(int scored, State[] sequence) {
        if (sequence == null)
            return State.NONE;
        int index = (scored+1) % 3;
        return sequence[index];
    }

    public static State get3rdNextToShoot(int scored, State[] sequence) {
        if (sequence == null)
            return State.NONE;
        int index = (scored+2) % 3;
        return sequence[index];
    }

    //Note: Due to some weird mapping and the fact that there is only 3 slots the function is self inverting and only one is needed
    //Note2: Only refers to juggler indexes and spinstates slots
    public static int convertJugglerIndexesAndSlots(int slotOrIndex) {
        switch (slotOrIndex) {
            case 0:  return 0;
            case 1:  return 2;
            case 2:  return 1;
            default: return -1;
        }
    }

    public synchronized int findClosestSlotOfState(int currentSlot, State desiredState) {
        if (!isStateInStates(desiredState)) return -1;
        else if (getSlot(currentSlot) == desiredState) return currentSlot;
        else {
            int[] ints = getIndexesOfStateInStates(desiredState);
            if (ints.length != 0) return ints[0];
            else return -1;
        }
    }

    public synchronized int findClosestJugglerIndexOfState(int jugglerIndex, State desiredState) {
        return convertJugglerIndexesAndSlots(findClosestSlotOfState(convertJugglerIndexesAndSlots(jugglerIndex), desiredState));
    }
}
