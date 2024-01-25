package org.firstinspires.ftc.teamcode.helpers;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.chesterlk.ftc.tweetybird.TweetyBirdProcessor;
import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.templates.TemplateProcessor;
import org.firstinspires.ftc.teamcode.vision.LineDetection;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class Hardwaremap {
    /**
     * Opmode References
     */
    private LinearOpMode currentOpmode;

    /**
     * Other used class references
     */
    //TweetyBird
    public TweetyBirdProcessor tweetyBird;
    public FtcDashboard dashboard = FtcDashboard.getInstance();

    /**
     * Global Enums
     */

    public enum DroperPosition {
        CLOSED,
        SINGLE,
        OPEN,
    }

    public enum ArmPosition {
        DOWN,
        UP
    }

    /**
     * Hardware Definitions
     */
    //Motor Section
    public DcMotor fl;
    public DcMotor bl;
    public DcMotor fr;
    public DcMotor br;

    public DcMotor intake;

    //Encoder Pods
    public DcMotor le,re,be;

    //Servos
    public Servo armL;
    public Servo armR;
    public Servo dropper;

    public CRServo lextend;
    public CRServo rextend;

    //Sensors
    public BNO055IMUNew imu = null;
    public WebcamName intakeCamera = null;

    //Temp memory
    private double yawOffset = 0;

    /**
     * Vision
     */
    public VisionPortal visionPortal;
    public AprilTagProcessor aprilTag;
    public TemplateProcessor cameraStreaming;
    public LineDetection lineDetection;


    /**
     * Constructor
     */
    public Hardwaremap(LinearOpMode opMode) {
        //Setting opmode
        currentOpmode = opMode;
    }

    /**
     * Initialize Method
     * This is the method that will be called to start the init process
     */
    public void init() {
        //Getting the current hardwaremap
        HardwareMap hwMap = currentOpmode.hardwareMap;

        //Initializing Motors
        fl = hwMap.get(DcMotor.class, "FL");
        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        bl = hwMap.get(DcMotor.class, "BL");
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fr = hwMap.get(DcMotor.class, "FR");
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        br = hwMap.get(DcMotor.class, "BR");
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake = hwMap.get(DcMotor.class, "Intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Initializing Encoders
        le = hwMap.get(DcMotor.class, "FR");
        le.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        le.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        re = hwMap.get(DcMotor.class, "FL");
        re.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        re.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        be = hwMap.get(DcMotor.class, "BL");
        be.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        be.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Servos
        armL = hwMap.get(Servo.class,"left arm");
        armL.setDirection(Servo.Direction.FORWARD);
        armR = hwMap.get(Servo.class,"right arm");
        armR.setDirection(Servo.Direction.FORWARD);
        dropper = hwMap.get(Servo.class,"dropper");
        dropper.setDirection(Servo.Direction.FORWARD);

        lextend = hwMap.get(CRServo.class,"Lextend");
        lextend.setDirection(DcMotorSimple.Direction.REVERSE);
        rextend = hwMap.get(CRServo.class,"Rextend");
        rextend.setDirection(DcMotorSimple.Direction.FORWARD);

        //Webcams
        intakeCamera = hwMap.get(WebcamName.class, "Webcam 1");

        //Initializing Imu
        imu = hwMap.get(BNO055IMUNew.class,"imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(parameters);

        //Initialize TweetyBird
        initTweetyBird();
        tweetyBird.disengage();
    }

    /**
     * Initialize TweetyBird Script
     * Only to be called by this class
     */
    private void initTweetyBird() {
        tweetyBird = new TweetyBirdProcessor.Builder()
                //Setting OpMode
                .setOpMode(currentOpmode)

                //Setitng Hardware Configuration
                .setFrontLeftMotor(fl)
                .setFrontRightMotor(fr)
                .setBackLeftMotor(bl)
                .setBackRightMotor(br)

                .setLeftEncoder(re)
                .setRightEncoder(le)
                .setMiddleEncoder(be)

                .flipLeftEncoder(true)
                .flipRightEncoder(true)
                .flipMiddleEncoder(true)

                .setInchesBetweenSideEncoders(11+(7.0/8.0))
                .setInchesToBackEncoder(-2)

                .setTicksPerEncoderRotation(2000)
                .setEncoderWheelRadius(1.88976/2.0)

                //Setting Up Basic Configuration
                .setMinSpeed(0.3)
                .setMaxSpeed(0.7)
                .setStartSpeed(0.27)
                .setSpeedModifier(0.04)
                .setStopForceSpeed(0.1)

                .setCorrectionOverpowerDistance(5)
                .setDistanceBuffer(1)
                .setRotationBuffer(8)

                //Build(create) TweetyBird
                .build();
        tweetyBird.disengage();
    }

    public void initVision() {
        lineDetection = new LineDetection(currentOpmode.telemetry);

        aprilTag = new AprilTagProcessor.Builder()
                .build();

        /*
        CameraName switchableCamera = ClassFactory.getInstance()
                .getCameraManager().nameForSwitchableCamera(frontCam, backCam);*/
        visionPortal = new VisionPortal.Builder()
                .setCamera(intakeCamera)
                .addProcessors(aprilTag, lineDetection)
                .setCameraResolution(new Size(640,480))
                .build();

        FtcDashboard.getInstance().startCameraStream(lineDetection,0);
    }

    /**
     * Easy Movement Controller
     */
    public void setMovementPower(double axial, double lateral, double yaw, double speed) {
        //Creating Individual Power for Each Motor
        double frontLeftPower  = ((axial + lateral + yaw) * speed);
        double frontRightPower = ((axial - lateral - yaw) * speed);
        double backLeftPower   = ((axial - lateral + yaw) * speed);
        double backRightPower  = ((axial + lateral - yaw) * speed);

        //Set Motor Power
        fl.setPower(frontLeftPower);
        fr.setPower(frontRightPower);
        bl.setPower(backLeftPower);
        br.setPower(backRightPower);
    }

    public void setDropperPosition(DroperPosition position) {
        if (position == DroperPosition.OPEN) {
            dropper.setPosition(0.05);
        }
        if (position == DroperPosition.CLOSED) {
            dropper.setPosition(0.9999);
        }
        if (position == DroperPosition.SINGLE) {
            dropper.setPosition(0.5);
        }
    }

    public void setArmPosition(ArmPosition position) {
        if(position==ArmPosition.DOWN) {
            if (armL.getPosition()==0) {
                armL.setPosition(0.17);
                armR.setPosition(0.15);
            }
            int step = 0;
            while (armL.getPosition()>=0.17) {
                if(step<3) {
                    step++;
                    armL.setPosition(armL.getPosition()-0.01);
                    armR.setPosition(armR.getPosition()-0.01);
                } else {
                    step = 0;
                    armL.setPosition(armL.getPosition()+0.01);
                    armR.setPosition(armR.getPosition()+0.01);
                }
                currentOpmode.sleep(5);
            }
        } else {
            armL.setPosition(0.8);
            armR.setPosition(0.78);
        }
    }

    /**
     * Yaw
     */
    public double getZ() {
        return tweetyBird.getZ()-yawOffset;
    }

    public void resetZ() {
        yawOffset = tweetyBird.getZ();
    }

}
