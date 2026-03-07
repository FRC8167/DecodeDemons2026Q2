package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.SensorColor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Cogintilities.MirrorUtility;
import org.firstinspires.ftc.teamcode.Commands.DetectArtifactCommand;
import org.firstinspires.ftc.teamcode.SubSystems.ColorMatch;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.SubSystems.Juggler;
import org.firstinspires.ftc.teamcode.SubSystems.JugglerAbsolute;
import org.firstinspires.ftc.teamcode.SubSystems.LimeLightVision;
import org.firstinspires.ftc.teamcode.SubSystems.LimitSwitch;
import org.firstinspires.ftc.teamcode.SubSystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.SubSystems.Popper;
import org.firstinspires.ftc.teamcode.SubSystems.RGBLight;
import org.firstinspires.ftc.teamcode.SubSystems.Shooter;
import org.firstinspires.ftc.teamcode.SubSystems.Slide;
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
    public static final Pose BLUE_SHOOT_FAR_POSE = new Pose(56, 14, Math.toRadians(110.5));
    private static final Pose RED_SHOOT_FAR_POSE = MirrorUtility.mirror(new Pose(56, 14, Math.toRadians(110.5)));
    private static final Pose RED_PARK_POSE = new Pose(38, 34.5, Math.toRadians(180));
    private static final Pose BLUE_PARK_POSE = new Pose(106, 34.5, Math.toRadians(0));
    private static final Pose NEUTRAL_PARK_POSE = new Pose(72, 34.5, Math.toRadians(90));

    protected static int initCount = 0;

    private static Alliance alliance = Alliance.UNSPECIFIED;

    public Telemetry telemetry;

    public Pose autoEndPose = null;
    public static OpModeType OP_MODE_TYPE;
    static List<LynxModule> ctrlHubs;

    public GoBildaPinpointDriver pinpoint;

    public Follower follower;
    public MecanumDrive mecanumDrive;
    public Intake intake;
    public Shooter shooter;
    public JugglerAbsolute juggler;
    public SensorColor colorSensor;
//    public Vision vision;
    public LimeLightVision vision;

    public Popper popper;
    public Slide slide;
    public RGBLight rgbLight;
    public ColorMatch colorMatch;
    public LimitSwitch limitSwitch;


    public void init(HardwareMap hardwareMap, boolean reset) throws InterruptedException {
        initCount ++;

        // Hardware
        MotorEx driveMotorRF = new MotorEx(hardwareMap, "RightFront").setCachingTolerance(0.01);
        MotorEx driveMotorLF = new MotorEx(hardwareMap, "LeftFront").setCachingTolerance(0.01);
        MotorEx driveMotorLR = new MotorEx(hardwareMap, "LeftRear").setCachingTolerance(0.01);
        MotorEx driveMotorRR = new MotorEx(hardwareMap, "RightRear").setCachingTolerance(0.01);

        follower = Constants.createFollower(hardwareMap);

        MotorEx intakeMotor    = new MotorEx(hardwareMap, "Intake").setCachingTolerance(0.01);
        MotorEx shooterMotor   = new MotorEx(hardwareMap, "Shooter").setCachingTolerance(0.01);
//        MotorEx spindexerMotor = new MotorEx(hardwareMap, "Juggler").setCachingTolerance(0.01);
//        MotorEx slideMotor = new MotorEx(hardwareMap, "Slide").setCachingTolerance(0.01);
        DcMotorEx slideMotor = hardwareMap.get(DcMotorEx.class, "Slide");

//        spindexerMotor.resetEncoder();  //added 01-18
//        slideMotor.resetEncoder();

        ServoEx popperServoL = new ServoEx(hardwareMap, "popperServoL");
        ServoEx popperServoR = new ServoEx(hardwareMap, "popperServoR");

        ServoEx rgbServo = new ServoEx(hardwareMap, "rgbServo");

        //WebcamName webCam1 = hardwareMap.get(WebcamName.class, "Webcam1");
        //sensorColor = new SensorColor(hardwareMap, "slot1Color");
        RevColorSensorV3 slot0Sensor = hardwareMap.get(RevColorSensorV3.class,"slot0Sensor");
        RevColorSensorV3 slot1Sensor = hardwareMap.get(RevColorSensorV3.class,"slot1Sensor");
        RevColorSensorV3 slot2Sensor = hardwareMap.get(RevColorSensorV3.class,"slot2Sensor");


        DigitalChannel jugglerDigitalSwitch = hardwareMap.get(DigitalChannel.class, "limitSwitch");
        jugglerDigitalSwitch.setMode(DigitalChannel.Mode.INPUT);
        limitSwitch = new LimitSwitch(jugglerDigitalSwitch);
//        juggler = new Juggler(spindexerMotor, limitSwitch);
        if (juggler == null || reset) {
            MotorEx spindexerMotor = new MotorEx(hardwareMap, "Juggler").setCachingTolerance(0.01);
            juggler = new JugglerAbsolute(spindexerMotor);
//            juggler = new Juggler(spindexerMotor, limitSwitch);
        }


        ctrlHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : ctrlHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }


        //Instantiate Subsystems
        mecanumDrive = new MecanumDrive(driveMotorLF, driveMotorLR, driveMotorRF, driveMotorRR);
        intake  = new Intake(intakeMotor);
        popper  = new Popper(popperServoL, popperServoR);
//        mecanumDrive = new MecanumDrive(driveMotorLF, driveMotorLR, driveMotorRF, driveMotorRR);
        slide = new Slide(slideMotor);
        shooter = new Shooter(shooterMotor);


//        vision = new Vision(hardwareMap.get(WebcamName.class, "Webcam1"));
        vision = new LimeLightVision(hardwareMap.get(Limelight3A.class, "limelight"));
        rgbLight   = new RGBLight(rgbServo);
        colorMatch = new ColorMatch(slot0Sensor, slot1Sensor, slot2Sensor);




        //Set default command for RGBLight using the registered colorMatch
        DetectArtifactCommand detectArtifactCommand =
                new DetectArtifactCommand(rgbLight, colorMatch, null); //, shooter);
        rgbLight.setDefaultCommand(detectArtifactCommand);

        //Register Subsystems
        register(mecanumDrive, intake, shooter, popper, slide, vision, rgbLight, colorMatch, juggler, limitSwitch);
        slide.setTargetTicks(8);

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

    public Pose getParkPose(){
        if (alliance == Alliance.BLUE) return BLUE_PARK_POSE;
        else if (alliance == Alliance.RED) return RED_PARK_POSE;
        else return NEUTRAL_PARK_POSE;  //default if UNSPECIFIED
    }


}





