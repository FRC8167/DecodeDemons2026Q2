package org.firstinspires.ftc.teamcode.TestOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Cogintilities.EricsCrap.BetterMotor;
import org.firstinspires.ftc.teamcode.Cogintilities.EricsCrap.ConfigurableConstants;
import org.firstinspires.ftc.teamcode.Cogintilities.EricsCrap.DefaultMotorInfo;
import org.firstinspires.ftc.teamcode.Cogintilities.TeamConstants;
import org.firstinspires.ftc.teamcode.Cogintilities.EricsCrap.TimedTimer;
//import org.firstinspires.ftc.teamcode.SubSystems.Lift;
//import org.firstinspires.ftc.teamcode.SubSystems.Motor1D;

@TeleOp(name="TeleOpTestPidTuner", group="Testing")
public class TeleOpTestPidTuner extends LinearOpMode implements TeamConstants {
    static TelemetryManager tmPanels;

    @Override
    public void runOpMode() throws InterruptedException {


//        initializeRobot(true);
        BetterMotor rawMotor = new BetterMotor(hardwareMap,"Juggler");
        DefaultMotorInfo.GOBILDA_117RPM.adjustMotor(rawMotor);

        rawMotor.setVelocityPIDFCoefficients(
                ConfigurableConstants.kP,
                ConfigurableConstants.kI,
                ConfigurableConstants.kD,
                ConfigurableConstants.kF
        );

        rawMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rawMotor.setTargetPosition(0);
        rawMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rawMotor.setPower(1);

        rawMotor.setVelocityRPM(20);


        tmPanels = PanelsTelemetry.INSTANCE.getTelemetry();
        double target = 0;
        TimedTimer timer = new TimedTimer(ConfigurableConstants.ALTERNATION_PERIOD/2.0);

//        tmPanels.addData("Target", target);
//        tmPanels.addData("Current", rawMotor.getVelocityRPM());
//        tmPanels.update(); //Note: Updating telemetry while using initializeRobot() will delete possible warnings

        waitForStart();

        while (opModeIsActive()) {

            if (timer.isDone()) {
                if (target == ConfigurableConstants.VEL_LOW_RPM) {
                    target = ConfigurableConstants.VEL_HIGH_RPM;
                } else if (target == ConfigurableConstants.VEL_HIGH_RPM) {
                    target = ConfigurableConstants.VEL_LOW_RPM;
                } else {
                    target = ConfigurableConstants.VEL_LOW_RPM;
                }
                rawMotor.setTargetPosition(target, AngleUnit.DEGREES);
                timer.startNewTimer(ConfigurableConstants.ALTERNATION_PERIOD/2.0);

                rawMotor.setVelocityPIDFCoefficients(
                        ConfigurableConstants.kP,
                        ConfigurableConstants.kI,
                        ConfigurableConstants.kD,
                        ConfigurableConstants.kF
                );

                rawMotor.setPositionPIDFCoefficients(
                        ConfigurableConstants.kP_Pos
                );
            }

            tmPanels.addData("Target", target);
            tmPanels.addData("Current", rawMotor.getVelocityRPM());
            tmPanels.addData("TargetDegs", rawMotor.getTargetPosition(AngleUnit.DEGREES));
            tmPanels.addData("CurrentDegs", rawMotor.getCurrentPosition(AngleUnit.DEGREES));
            tmPanels.addData("CurrentTicks", rawMotor.getCurrentPosition());
            tmPanels.addData("CountsPerRev", rawMotor.getMotorType().getTicksPerRev());
            tmPanels.update();
        }
    }
}
