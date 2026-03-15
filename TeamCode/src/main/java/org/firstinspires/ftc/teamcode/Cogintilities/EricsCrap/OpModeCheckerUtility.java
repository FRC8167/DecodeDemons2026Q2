package org.firstinspires.ftc.teamcode.Cogintilities.EricsCrap;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

public class OpModeCheckerUtility {
    private OpModeCheckerUtility() {}

    public static boolean isOpMode(@NonNull Class<?> clazz) {
        return clazz.isAnnotationPresent(Autonomous.class) || clazz.isAnnotationPresent(TeleOp.class);
    }

    public static boolean isAutonomous(@NonNull Class<?> clazz) {
        return clazz.isAnnotationPresent(Autonomous.class);
    }

    public static boolean isTeleOp(@NonNull Class<?> clazz) {
        return clazz.isAnnotationPresent(TeleOp.class);
    }

    @Nullable
    public static Robot.OpModeType getOpModeType(@NonNull Class<?> clazz) {
        if (isAutonomous(clazz)) {
            return Robot.OpModeType.AUTO;
        } else if (isTeleOp(clazz)) {
            return Robot.OpModeType.TELEOP;
        } else {
            return null;
        }
    }

}
