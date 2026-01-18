package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Gamepad;



import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.SubSystems.ColorMatch;
import org.firstinspires.ftc.teamcode.SubSystems.RGBLight;


public class DetectArtifactCommand extends CommandBase {

    private final RGBLight rgbLight;
    private final ColorMatch colorMatch;
    private final Gamepad operator;
    Robot robot = Robot.getInstance();

    private final ElapsedTime flashTimer = new ElapsedTime();
    private final double flashInterval = 0.5; // seconds

    private boolean lightOn = true;
    private boolean hasRumbled = false;

    private RGBLight.LightColor currentColor = RGBLight.LightColor.OFF;

    public DetectArtifactCommand(RGBLight rgbLight, ColorMatch colorMatch, Gamepad operator) { //, Shooter shooter) {
        this.rgbLight = rgbLight;
        this.colorMatch = colorMatch;
        this.operator = operator;
        addRequirements(rgbLight); // only RGBLight is a hardware subsystem
    }

    @Override
    public void initialize() {
        flashTimer.reset();
    }

    @Override
    public void execute() {
                ColorMatch.ArtifactColor detected =
                colorMatch.detectColor(ColorMatch.Slot.SLOT_0);

        switch (detected) {
            case GREEN:
                currentColor = RGBLight.LightColor.GREEN;
                break;
            case PURPLE:
                currentColor = RGBLight.LightColor.VIOLET;
                break;
            default:
                currentColor = RGBLight.LightColor.BLUE;
                break;
        }

        if (robot.shooter.atTargetVelocity()) {

            if (Robot.OP_MODE_TYPE == Robot.OpModeType.AUTO) {
                // AUTO: flash LED
                if (flashTimer.seconds() >= flashInterval) {
                    lightOn = !lightOn;
                    flashTimer.reset();
                }

            } else {
                // TELEOP: rumble once + solid LED
                if (!hasRumbled && operator != null) {
                    operator.rumble(200);
                    hasRumbled = true;
                }
                lightOn = true;
            }

            if (lightOn) {
                rgbLight.setColor(currentColor);
            } else {
                rgbLight.off();
            }

        } else {
            // Shooter not ready → solid color, reset rumble latch
            rgbLight.setColor(currentColor);
            hasRumbled = false;
        }
    }


    @Override
    public boolean isFinished() {
        return false; //always running
    }

    @Override
    public void end(boolean interrupted) {
        rgbLight.off();
    }
}
