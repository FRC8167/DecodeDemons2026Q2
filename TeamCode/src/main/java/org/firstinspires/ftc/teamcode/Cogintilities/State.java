package org.firstinspires.ftc.teamcode.Cogintilities;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.SubSystems.ColorMatch;
import org.jetbrains.annotations.Contract;

import java.util.ArrayList;
import java.util.List;

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

    public ColorMatch.ArtifactColor revert() {
        return revert(this);
    }

    @Contract(pure = true)
    public static ColorMatch.ArtifactColor revert(@NonNull State state) {
        switch (state) {
            case GREEN:
                return ColorMatch.ArtifactColor.GREEN;
            case PURPLE:
                return ColorMatch.ArtifactColor.PURPLE;
            case UNKNOWN:
                return ColorMatch.ArtifactColor.UNKNOWN;
            default:
                return ColorMatch.ArtifactColor.NONE;
        }
    }

    @Contract(pure = true)
    public static State migrate(@NonNull ColorMatch.ArtifactColor color) {
        switch (color) {
            case GREEN:
                return GREEN;
            case PURPLE:
                return PURPLE;
            case UNKNOWN:
                return UNKNOWN;
            default:
                return NONE;
        }
    }

    public static State[] sequenceMigrate(@NonNull ColorMatch.ArtifactColor... colors) {
        List<State> states = new ArrayList<>();
        for (ColorMatch.ArtifactColor color: colors) {
            states.add(migrate(color));
        }
        return states.toArray(new State[0]);
    }

    public static ColorMatch.ArtifactColor[] sequenceRevert(@NonNull State... states) {
        List<ColorMatch.ArtifactColor> colors = new ArrayList<>();
        for (State state: states) {
            colors.add(state.revert());
        }
        return colors.toArray(new ColorMatch.ArtifactColor[0]);
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
