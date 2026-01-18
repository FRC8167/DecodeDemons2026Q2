package org.firstinspires.ftc.teamcode.Commands;
import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubSystems.Vision;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class VisionCommand extends CommandBase {
    private final Vision vision;

    public VisionCommand(Vision vision) {
        this.vision = vision;
//        addRequirements(vision);
    }

    @Override
    public void initialize() {
//        vision.enableAprilTagDetection();
    }

    @Override
    public void execute() {
        vision.scanForAprilTags();
    }

    @Override
    public void end(boolean interrupted) {
//        vision.disableAprilTagDetection();
    }

    @Override
    public boolean isFinished() {
        return false;  // runs until you cancel it with a button
    }
}
