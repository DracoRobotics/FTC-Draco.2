package org.firstinspires.ftc.teamcode.extra;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class DemonstrationCode extends OpMode {

    // ---------------- HARDWARE ----------------
    private Limelight3A limelight;
    private IMU imu;
    private DcMotor turret;


    private DcMotor leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive;
    private DcMotor intakeMotor;
    private DcMotorEx flywheelLeft, flywheelRight;
    private Servo indexer;


    // ---------------- TIMERS ----------------
    private ElapsedTime indexerTimer = new ElapsedTime();
    private ElapsedTime shooterTimer = new ElapsedTime();


    // ---------------- VISION ----------------
    private LLResult limelightResult;


    // ---------------- INDEXER ----------------
    private enum IndexerState { CLOSED, OPENING, OPEN, CLOSING }


    private static final double INDEXER_OPEN_POS = 0.8;
    private static final double INDEXER_CLOSED_POS = 0.5;
    private static final double INDEXER_MOVE_TIME = 0.25;


    // ---------------- SHOOTER ----------------
    private enum ShooterState { IDLE, SPINNING_UP, FEEDING, CLOSE_GATE }


    private static final double FEED_TIME = 3.0;


    // Flywheel constants
    private static final double TICKS_PER_REV = 28.0;
    private static final double RPM_TOLERANCE = 75.0;
    private static final double MIN_SPINUP_TIME = 0.3;


    private boolean shootButtonLast = false;


    // ---------------- TURRET PID ----------------
    private static final double TICKS_PER_DEGREE = 9.744444;
    private static final double MAX_TURN_POWER = 1.0;


    private double Kp = 0.000059;
    private double Ki = 0.000055;
    private double Kd = 0.0002;


    private double integralSum = 0.0;
    private double lastError = 0.0;
    private long lastTimeMs = 0;


    private double lastFilteredTx = 0.0;
    private boolean headingLockEnabled = false;





    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);


        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP)));


        turret = hardwareMap.get(DcMotor.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        leftFrontDrive  = hardwareMap.get(DcMotor.class, "frontleft");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "backleft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontright");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "backright");


        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);


        intakeMotor = hardwareMap.get(DcMotor.class, "intake");


        flywheelLeft  = hardwareMap.get(DcMotorEx.class, "flywheelLeft");
        flywheelRight = hardwareMap.get(DcMotorEx.class, "flywheelRight");


        // 🔥 CRITICAL CHANGE
        flywheelLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        indexer = hardwareMap.get(Servo.class, "indexer");
        indexer.setPosition(INDEXER_CLOSED_POS);

    }

    @Override
    public void loop() {


        if (gamepad1.a){
            intakeMotor.setPower(-1);
        }

        else if (gamepad1.b){
            intakeMotor.setPower(0);
        }

        if (gamepad1.left_bumper){
            flywheelLeft.setPower(1);
            flywheelRight.setPower(1);
        }
        else {
            flywheelLeft.setPower(0);
            flywheelRight.setPower(0);
        }



    }
}
