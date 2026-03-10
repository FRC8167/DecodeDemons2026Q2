//package org.firstinspires.ftc.teamcode.TestOpModes;
//
//import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.TelemetryManager;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
//
//import org.firstinspires.ftc.teamcode.SubSystems.Slide;
//
//
//@Configurable
//@TeleOp(name = "Slider PID Tuning", group="TestOps")
//public class TestOp_SlideTuning extends OpMode {
//
//    Slide slide;
//
//    static int cmdCounts;
//    static int MAX_COUNTS = (int)(537.7 * 0.7);
//    static int MIN_COUNTS = (int)(537.7 * 0.4);
//    static long STEP_DURATION_SEC = 2;
//
//    static TelemetryManager tmPanels;
//
//    long currentTime = 0;
//    long prevTime = 0;
//
//    private enum State {HIGH, LOW}
//    State nextState = State.HIGH;
//
//
//    @Override
//    public void init() {
//        MotorEx slideMotor = new MotorEx(hardwareMap, "Slide").setCachingTolerance(0.01);
//        slide = new Slide(slideMotor);
//        tmPanels = PanelsTelemetry.INSTANCE.getTelemetry();
//    }
//
//
//    @Override
//    public void init_loop() {
//        super.init_loop();
//    }
//
//
//    @Override
//    public void start() {
//        prevTime = System.currentTimeMillis();
//    }
//
//
//    @Override
//    public void loop() {
//
//        slide.periodic();
//        currentTime = System.currentTimeMillis();
//
//        // Create square wave command between 20%-80% of motor full speed rpm. 10s High and 10s low
//        if ((currentTime - prevTime) >= (STEP_DURATION_SEC * 1000)) {
//            switch (nextState) {
//                case HIGH:
//                    cmdCounts = MAX_COUNTS;
//                    nextState = State.LOW;
//                    break;
//
//                case LOW:
//                    cmdCounts = MIN_COUNTS;
//                    nextState = State.HIGH;
//                    break;
//            }
//            slide.setTargetTicks(cmdCounts);
//            prevTime = currentTime;
//        }
//
//
//        //Allow for Plots
//        tmPanels.debug(cmdCounts);
//        tmPanels.debug(slide.getPositionTicks());
//
//        // Display on Panels
//        tmPanels.addData("Commanded Position (cnts)", cmdCounts);
//        tmPanels.addData("Actual Position (cnts)", slide.getPositionTicks());
//        tmPanels.addData("Slide at Target ", slide.atTarget());
//
//        tmPanels.update(telemetry);     // Should update both the driver station and panels
//    }
//}
//
