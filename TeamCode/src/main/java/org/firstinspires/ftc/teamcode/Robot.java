package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.SensorColor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Cogintilities.MirrorUtility;
import org.firstinspires.ftc.teamcode.Commands.DetectArtifactCommand;
import org.firstinspires.ftc.teamcode.SubSystems.ColorMatch;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.SubSystems.Juggler;
import org.firstinspires.ftc.teamcode.SubSystems.LimeLightVision;
import org.firstinspires.ftc.teamcode.SubSystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.SubSystems.Popper;
import org.firstinspires.ftc.teamcode.SubSystems.RGBLight;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter;
import org.firstinspires.ftc.teamcode.SubSystems.Vision;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.follower.Follower;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;


import java.util.List;


public class Robot extends com.seattlesolvers.solverslib.command.Robot {

    private static final Robot instance = new Robot();
    public static Robot getInstance() {
        return instance;
    }

    public enum OpModeType {
        AUTO,
        TELEOP
    }

    public enum Alliance {
        RED,
        BLUE,
        UNSPECIFIED
    }

    //field poses
    public static final Pose BLUE_SHOOT_FAR_POSE = MirrorUtility.mirror(new Pose(56, 12, Math.toRadians(-72)));
    private static final Pose RED_SHOOT_FAR_POSE = MirrorUtility.mirror(new Pose(56, 12, Math.toRadians(-72)));

    private Alliance alliance = Alliance.UNSPECIFIED;

    public Telemetry telemetry;


    public Pose autoEndPose = null;
    public static OpModeType OP_MODE_TYPE;
    static List<LynxModule> ctrlHubs;

    public MotorEx driveMotorRF;
    public MotorEx driveMotorLF;
    public MotorEx driveMotorRR;
    public MotorEx driveMotorLR;

    public MotorEx intakeMotor;
    public MotorEx shooterMotor;
    public MotorEx spindexerMotor;
    public ServoEx popperServoL;
    public ServoEx popperServoR;

//    public SensorColor sensorColor;
    public SensorColor slot0Sensor;
    public SensorColor slot1Sensor;
    public SensorColor slot2Sensor;
    public Follower follower;

    public WebcamName webCam1;
//    public Limelight3A limelight;

    public GoBildaPinpointDriver pinpoint;


    public MecanumDrive mecanumDrive;


    public Intake intake;
    public Shooter shooter;
    public Juggler juggler;
    public SensorColor colorSensor;

    public Vision vision;
    public LimeLightVision limey;

//    public Feeder feederF, feederR;
//    public Gate gate;
    public Popper popper;
    public RGBLight rgbLight;
    public ColorMatch colorMatch;


    public void init(HardwareMap hardwareMap) throws InterruptedException {


        // Hardware
        driveMotorRF = new MotorEx(hardwareMap, "RightFront").setCachingTolerance(0.01);
        driveMotorLF = new MotorEx(hardwareMap, "LeftFront").setCachingTolerance(0.01);
        driveMotorLR = new MotorEx(hardwareMap, "LeftRear").setCachingTolerance(0.01);
        driveMotorRR = new MotorEx(hardwareMap, "RightRear").setCachingTolerance(0.01);

        follower = Constants.createFollower(hardwareMap);

        intakeMotor = new MotorEx(hardwareMap, "Intake").setCachingTolerance(0.01);
        shooterMotor = new MotorEx(hardwareMap, "Shooter").setCachingTolerance(0.01);
        spindexerMotor = new MotorEx(hardwareMap, "Juggler").setCachingTolerance(0.01);

//        CRServo feederServoF = new CRServo(hardwareMap, "feederServoF");
//        CRServo feederServoR = new CRServo(hardwareMap, "feederServoR");
        ServoEx popperServoL = new ServoEx(hardwareMap, "popperServoL");
        ServoEx popperServoR = new ServoEx(hardwareMap, "popperServoR");

//        ServoEx gateServo = new ServoEx(hardwareMap, "gateServo");

        ServoEx rgbServo = new ServoEx(hardwareMap, "rgbServo");

        webCam1 = hardwareMap.get(WebcamName.class, "Webcam1");
        //limelight = hwMap.get(Limelight3A.class, "limelight");  //dreaming
        //sensorColor = new SensorColor(hardwareMap, "slot1Color");
        slot0Sensor = new SensorColor(hardwareMap, "slot0Sensor");
        slot1Sensor = new SensorColor(hardwareMap, "slot1Sensor");
        slot2Sensor = new SensorColor(hardwareMap, "slot2Sensor");

        ctrlHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : ctrlHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }


        //Instantiate Subsystems
        mecanumDrive = new MecanumDrive(driveMotorLF, driveMotorLR, driveMotorRF, driveMotorRR);
        intake  = new Intake(intakeMotor);
//        feederF  = new Feeder(feederServoF);
//        feederR = new Feeder(feederServoR);
        popper= new Popper(popperServoL, popperServoR);
        shooter = new Shooter(shooterMotor);
        juggler = new Juggler(spindexerMotor);
        spindexerMotor.resetEncoder();  //added 01-18

        vision  = new Vision(webCam1);
//        limey = new LimeLightVision(hardwareMap.get(Limelight3A.class, "limelight"), 1, true);

        rgbLight = new RGBLight(rgbServo);
        colorMatch = new ColorMatch(slot0Sensor, slot1Sensor, slot2Sensor);
//        gate = new Gate(gateServo);



        //Set default command for RGBLight using the registered colorMatch
        DetectArtifactCommand detectArtifactCommand =
                new DetectArtifactCommand(rgbLight, colorMatch, null); //, shooter);
        rgbLight.setDefaultCommand(detectArtifactCommand);

        //Register Subsystems
        register(mecanumDrive, intake, shooter, popper, vision, rgbLight, colorMatch, juggler);

        if (OP_MODE_TYPE.equals(OpModeType.AUTO)) {
            initHasMovement();
        }
    }

    public void initHasMovement() {
        //TODO what goes here??
    }

    public void setAlliance(Alliance ally) {
        alliance = ally;
    }

    public Alliance getAlliance() {
        return alliance;
    }

    public Pose getShootPose(){
        if (alliance == Alliance.BLUE) return BLUE_SHOOT_FAR_POSE;
        else if (alliance == Alliance.RED) return RED_SHOOT_FAR_POSE;
        else return BLUE_SHOOT_FAR_POSE;  //default if UNSPECIFIED
    }


}





