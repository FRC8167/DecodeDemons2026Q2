package org.firstinspires.ftc.teamcode.Cogintilities;

import androidx.annotation.NonNull;

import org.jetbrains.annotations.Contract;

public enum State {
    GREEN("Green", 'G'),
    PURPLE("Purple", 'P'),
    UNKNOWN("Unknown", 'U'),
    NONE("None", 'N');
    private final String color;
    private final char character;

    State(String color, char character) {
        this.color = color;
        this.character = character;
    };
    public String getColor() {
        return color;
    }
    public char getCharacter() {
        return character;
    }

    @NonNull
    public static String convertStatesToInitials(State... states) {
        if (states == null) {
            return "";
        } else {
            StringBuilder string = new StringBuilder();
            for (State state : states) {
                string.append(state.getCharacter());
            }
            return string.toString();
        }
    }

    @NonNull
    @Contract(value = " -> new", pure = true)
    public static State[] GPP() {
        return new State[]{GREEN, PURPLE, PURPLE};
    }

    @NonNull
    @Contract(value = " -> new", pure = true)
    public static State[] PGP() {
        return new State[]{PURPLE, GREEN, PURPLE};
    }

    @NonNull
    @Contract(value = " -> new", pure = true)
    public static State[] PPG() {
        return new State[]{PURPLE, PURPLE, GREEN};
    }

} // Slot states for spindexer
