package org.firstinspires.ftc.teamcode.extra;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantFunction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Configuration.MecanumDrive;


@Autonomous
public class BANEAutonRB extends LinearOpMode {


    // ===== Hardware =====
    DcMotor intakeMotor;
    DcMotor flywheelLeft, flywheelRight;
    Servo indexer;


    // ===== Shooter State Machine =====
    public void intakeRun() {
        intakeMotor.setPower(-1);
    }


    public void intakeStop() {
        intakeMotor.setPower(0);
    }
    public void Outtake() {
        intakeMotor.setPower(1);
    }







    public void SpinFlywheel(){
        flywheelLeft.setPower(0.6);
        flywheelRight.setPower(0.6);


    }


    public void StopFlywheel (){
        flywheelLeft.setPower(0);
        flywheelRight.setPower(0);
    }


    public void IndexerOpen(){
        indexer.setPosition(0.8);

    }
    public void IndexerClose(){
        indexer.setPosition(0.5);
    }

    public void ThreeBallShoot (){
        IndexerOpen();
        SpinFlywheel();
        sleep(700);
        intakeRun();
        sleep(4000);
        intakeStop();
        IndexerClose();
        StopFlywheel();
    }





    // ===== Road Runner Instant Actions =====
    public class ThreeBallShoot implements InstantFunction {
        @Override
        public void run() {
            ThreeBallShoot();
        }
    }


    public class IntakeOn implements InstantFunction {
        @Override
        public void run() {
            intakeMotor.setPower(-1);
        }
    }


    public class IntakeOff implements InstantFunction {
        @Override
        public void run() {
            intakeMotor.setPower(0);
        }
    }


    @Override
    public void runOpMode() {


        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        flywheelLeft = hardwareMap.get(DcMotor.class, "flywheelLeft");
        flywheelRight = hardwareMap.get(DcMotor.class, "flywheelRight");
        indexer = hardwareMap.get(Servo.class, "indexer");


        flywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        Pose2d beginPose = new Pose2d(new Vector2d(-51, -52), Math.toRadians(180)); // Starting position
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose); // MecanumDrive setup

// ===== TRAJECTORY (UPDATED) =====
        Action myAuton = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(-15, -15))
                .stopAndAdd(new ThreeBallShoot())


                .splineTo(new Vector2d(35, -30), Math.toRadians(270))
                .stopAndAdd(new IntakeOn())
                .lineToY(-60)
                .lineToY(-30)


                .splineTo(new Vector2d(-15, -15), Math.toRadians(45))
                .stopAndAdd(new IntakeOff())
                .turnTo(Math.toRadians(225))
                .stopAndAdd(new ThreeBallShoot())


                .splineTo(new Vector2d(11.9, -30), Math.toRadians(270))
                .stopAndAdd(new IntakeOn())
                .lineToY(-60)
                .lineToY(-30)



                .splineTo(new Vector2d(-15,-15), Math.toRadians(45))
                .stopAndAdd(new IntakeOff())
                .turnTo(Math.toRadians(225))
                .stopAndAdd(new ThreeBallShoot())


                .build();

        waitForStart(); // Wait for the start signal

        Actions.runBlocking(new SequentialAction(
                myAuton
        ));
    }
}

