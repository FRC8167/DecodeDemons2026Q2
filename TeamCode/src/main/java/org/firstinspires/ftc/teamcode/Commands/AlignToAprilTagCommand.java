package org.firstinspires.ftc.teamcode.Commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.SubSystems.LimeLightVision;

public class AlignToAprilTagCommand extends CommandBase {
    private final Follower follower;
    private final LimeLightVision vision;
    private final double TOLERANCE = 1; // Degrees

    public AlignToAprilTagCommand(Follower follower, LimeLightVision vision) {
        this.follower = follower;
        this.vision = vision;
    }

    @Override
    public void execute() {
        if (vision.hasTarget()) {
            // getTX() returns the degrees off-center
            double tx = vision.getTX();

            // Optimization: Apply the correction to the follower's current pose
            Pose currentPose = follower.getPose();
            double correctedHeading = currentPose.getHeading() + Math.toRadians(tx);

            // Update Pedro Pathing with the 'Absolute Truth' from the Limelight
            follower.setPose(new Pose(
                    currentPose.getX(),
                    currentPose.getY(),
                    correctedHeading
            ));
        }
    }

    @Override
    public boolean isFinished() {
        // Command finishes once the horizontal offset is negligible
        return Math.abs(vision.getTX()) < TOLERANCE || !vision.hasTarget();
    }
}

