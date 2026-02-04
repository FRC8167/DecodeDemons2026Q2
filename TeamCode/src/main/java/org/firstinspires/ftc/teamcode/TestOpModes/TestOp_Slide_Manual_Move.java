package org.firstinspires.ftc.teamcode.TestOpModes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.SubSystems.Slide;


@Configurable
@TeleOp(name="Slide Manual Move", group="TestOps")
public class TestOp_Slide_Manual_Move extends OpMode {

    static TelemetryManager tmPanels;

    public  static int cmdCounts;
    private double largeIncrement = 20;
    private double smallINcrement = 5;
    private double fineIncrement = 1;

    boolean xPressed, yPressed, aPressed, bPressed, right_bumperPressed, left_bumperPressed;

    Slide slide;

    @Override
    public void init() {
        MotorEx slideMotor = new MotorEx(hardwareMap, "Slide");

        slide  = new Slide(slideMotor);
        tmPanels = PanelsTelemetry.INSTANCE.getTelemetry();

        xPressed = false;
        yPressed = false;
        aPressed = false;
        bPressed = false;
        right_bumperPressed = false;
        left_bumperPressed = false;

        cmdCounts = 10;
    }


    @Override
    public void loop() {

        slide.periodic();

        if(gamepad1.x & !xPressed) {
            cmdCounts -= largeIncrement;
        }

        if(gamepad1.y & !yPressed) {
            cmdCounts += largeIncrement;
        }

        if(gamepad1.a & !aPressed) {
            cmdCounts -= smallINcrement;
        }

        if(gamepad1.b & !bPressed) {
            cmdCounts += smallINcrement;
        }

        if(gamepad1.left_bumper & !left_bumperPressed) {
            cmdCounts -= fineIncrement;
        }

        if(gamepad1.right_bumper & !right_bumperPressed) {
            cmdCounts += fineIncrement;
        }

        slide.setTargetTicks(cmdCounts);

        telemetry.addLine("Using Gamepad 1");
        telemetry.addLine("Increment by 20:  X-, Y+");
        telemetry.addLine("Increment by  5:  A-, B+");
        telemetry.addLine("Increment by  1: LB-, RB+");
        telemetry.addLine("*************************");
        telemetry.addData("Commanded Position [cnts]", cmdCounts);
        telemetry.addData("Actual Position [cnts]", slide.getPositionTicks());
        telemetry.addData("Slide at Target", slide.atTarget());
        telemetry.addData("Error [cnts]", cmdCounts - slide.getPositionTicks());
        telemetry.update();

        tmPanels.addData("Commanded Position [cnts]", cmdCounts);
        tmPanels.addData("Actual Position [cnts]", slide.getPositionTicks());
        tmPanels.addData("Slide at Target", slide.atTarget());
        tmPanels.addData("Error [cnts]", cmdCounts - slide.getPositionTicks());
        tmPanels.update();

        xPressed = gamepad1.x;
        yPressed = gamepad1.y;
        aPressed = gamepad1.a;
        bPressed = gamepad1.b;
        left_bumperPressed = gamepad1.left_bumper;
        right_bumperPressed = gamepad1.right_bumper;

    }


}
