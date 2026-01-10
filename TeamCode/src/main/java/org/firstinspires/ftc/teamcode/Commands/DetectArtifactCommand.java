package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.SubSystems.ColorMatch;
import org.firstinspires.ftc.teamcode.SubSystems.RGBLight;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter;

public class DetectArtifactCommand extends CommandBase {

    private final RGBLight rgbLight;
    private final ColorMatch colorMatch;
    //private final Shooter shooter;
    Robot robot = Robot.getInstance();

    private final ElapsedTime flashTimer = new ElapsedTime();
    private final double flashInterval = 0.5; // seconds

    private boolean lightOn = true;
    private RGBLight.LightColor currentColor = RGBLight.LightColor.OFF;

    public DetectArtifactCommand(RGBLight rgbLight, ColorMatch colorMatch) { //, Shooter shooter) {
        this.rgbLight = rgbLight;
        this.colorMatch = colorMatch;
        //this.shooter = shooter;
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
            case RED:
                currentColor = RGBLight.LightColor.RED;
                break;
            case UNKNOWN:
            default:
                currentColor = RGBLight.LightColor.BLUE;
                break;
        }



        if (robot.shooter.atTargetVelocity()) {
            // flash LED
            if (flashTimer.seconds() >= flashInterval) {
                lightOn = !lightOn; // toggle
                flashTimer.reset();
            }
            if (lightOn) {
                rgbLight.setColor(currentColor);
            } else {
                rgbLight.off();
            }
        } else {
            // LED solid
            rgbLight.setColor(currentColor);
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
