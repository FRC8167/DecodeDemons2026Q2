package org.firstinspires.ftc.teamcode.TestOpModes;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Cogintilities.EricsCrap.BetterMotor;
import org.firstinspires.ftc.teamcode.Cogintilities.TeamConstants;
import org.firstinspires.ftc.teamcode.SubSystems.JugglerAbsolute;
//import org.firstinspires.ftc.teamcode.SubSystems.Lift;
//import org.firstinspires.ftc.teamcode.SubSystems.Motor1D;

@TeleOp(name="TeleOpTestNewJuggler", group="Testing")
public class TeleOpTestNewJuggler extends LinearOpMode implements TeamConstants {
    static TelemetryManager tmPanels;

    @Override
    public void runOpMode() throws InterruptedException {



        BetterMotor motor = new BetterMotor(hardwareMap,"Juggler");
        JugglerAbsolute juggler = new JugglerAbsolute(motor);

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("Index", juggler.getSlotIndex());
            telemetry.addData("Pos", motor.getCurrentPosition());
            telemetry.update();
        }
    }
}
