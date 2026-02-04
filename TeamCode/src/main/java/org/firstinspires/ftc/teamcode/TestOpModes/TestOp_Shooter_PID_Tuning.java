package org.firstinspires.ftc.teamcode.TestOpModes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.SubSystems.Shooter_Alternate;

@Configurable
@TeleOp(name = "Shooter PID Tuning", group="TestOps")
public class TestOp_Shooter_PID_Tuning extends OpMode {

    Shooter_Alternate shooter;

    static double cmd;
    static double MAX_MOTOR_RPM     = 6000;
    static long   STEP_DURATION_SEC = 10;

    static TelemetryManager tmPanels;

    long currentTime = 0;
    long prevTime = 0;

    private enum State {HIGH, LOW};
    State nextState = State.HIGH;


    @Override
    public void init() {
        MotorEx shooterMotor = new MotorEx(hardwareMap, "Shooter").setCachingTolerance(0.01);
        shooter  = new Shooter_Alternate(shooterMotor);
        tmPanels = PanelsTelemetry.INSTANCE.getTelemetry();
    }


    @Override
    public void init_loop() {
        super.init_loop();
    }


    @Override
    public void start() {
        prevTime = System.currentTimeMillis();
    }


    @Override
    public void loop() {

        shooter.periodic();
        currentTime = System.currentTimeMillis();

        // Create square wave command between 20%-80% of motor full speed rpm. 10s High and 10s low
        if( (currentTime - prevTime) >= (STEP_DURATION_SEC * 1000) ) {
            switch (nextState) {
                case HIGH:
                    cmd = MAX_MOTOR_RPM * 0.80;
                    nextState = State.LOW;
                    break;

                case LOW:
                    cmd = MAX_MOTOR_RPM * 0.20;
                    nextState = State.HIGH;
                    break;
            }
            shooter.setVelocity(cmd);
            prevTime = currentTime;
        }


        // Display on Panels
        tmPanels.debug("Commanded Speed (RPM)",       cmd);
        tmPanels.debug("Shooter Speed (RPM)", "%.1f", shooter.getRPM());
        tmPanels.debug("Shooter Speed (TPS)", "%.1f", shooter.getTicsPerSec());
        tmPanels.debug("Shooter at Target ",          shooter.atTargetVelocity());

        tmPanels.addData("------- Using addData ", "instead of debug -------");
        tmPanels.addData("Commanded RPM", cmd);
        tmPanels.addData("Motor RPM",     shooter.getRPM());

        tmPanels.update(telemetry);     // Should update both the driver station and panels

    }
}



