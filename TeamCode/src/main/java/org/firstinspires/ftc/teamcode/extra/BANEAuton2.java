package org.firstinspires.ftc.teamcode.extra;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@Autonomous
public class BANEAuton2 extends LinearOpMode {


    private Limelight3A limelight;
    private IMU imu;
    private DcMotor turret;

    private DcMotor leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive;
    private DcMotor intakeMotor;
    private DcMotor flywheelLeft, flywheelRight;
    private Servo indexer;


    public void DriveForward() {
        leftBackDrive.setPower(1);
        rightBackDrive.setPower(1);
        rightFrontDrive.setPower(1);
        leftFrontDrive.setPower(1);
    }

    public void DriveBackward (){
        leftBackDrive.setPower(-1);
        rightBackDrive.setPower(-1);
        rightFrontDrive.setPower(-1);
        leftFrontDrive.setPower(-1);
    }

    public void TurnRight() {
        leftBackDrive.setPower(1);
        rightBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftFrontDrive.setPower(1);
    }


    public void TurnLeft (){
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(1);
        rightFrontDrive.setPower(1);
        leftFrontDrive.setPower(0);
    }

    public void DriveStop (){
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftFrontDrive.setPower(0);
    }

    public void SpinFlywheel(){
        flywheelLeft.setPower(0.55);
        flywheelRight.setPower(0.55);

    }

    public void StopFlywheel (){
        flywheelLeft.setPower(0);
        flywheelRight.setPower(0);
    }

    public void ShootBall(){
        sleep(100);
        indexer.setPosition(1);
        sleep(800);
        indexer.setPosition(0.4);
        sleep(800);

    }

    public void ThreeBallShoot (){
        SpinFlywheel();
        sleep(1200);
        ShootBall();
        intakeRun();
        sleep(200);
        intakeStop();
        ShootBall();
        sleep(100);
        intakeRun();
        sleep(500);
        ShootBall();
        StopFlywheel();
        intakeStop();
    }

    public void intakeRun() {
        intakeMotor.setPower(-1);
    }

    public void intakeStop() {
        intakeMotor.setPower(0);
    }



    @Override
    public void runOpMode() throws InterruptedException {
// --- LIMELIGHT ---
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);

        // --- IMU ---
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(orientation));

        // --- TURRET ---
        turret = hardwareMap.get(DcMotor.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // --- DRIVE ---
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "frontleft");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "backleft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontright");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "backright");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --- INTAKE & SHOOTER ---
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");

        flywheelLeft = hardwareMap.get(DcMotor.class, "flywheelLeft");
        flywheelRight = hardwareMap.get(DcMotor.class, "flywheelRight");

        flywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        indexer = hardwareMap.get(Servo.class, "indexer");
        indexer.setPosition(0.4);

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();


        DriveBackward();
        sleep(850);
        DriveStop();
        ThreeBallShoot();
        TurnRight();
        sleep(500);
        DriveStop();
        intakeRun();
        sleep(100);
        DriveForward();
        sleep(600);
        DriveStop();
        sleep(200);
        DriveBackward();
        sleep(700);
        DriveStop();
        TurnLeft();
        sleep(650);
        DriveStop();
        intakeStop();
        ThreeBallShoot();
        DriveBackward();
        sleep(500);
        DriveStop();



    }
}
