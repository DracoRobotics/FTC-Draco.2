package org.firstinspires.ftc.teamcode.Competition;


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
public class OrionRED extends LinearOpMode {


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
        sleep(900);
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


        Pose2d beginPose = new Pose2d(new Vector2d( 65, 12), Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);


        // ===== TRAJECTORY (UNCHANGED) =====
        Action BANEAuton = drive.actionBuilder(beginPose)
                .splineToConstantHeading(new Vector2d(-15, 15), Math.toRadians(180))
                .turnTo(Math.toRadians(-225))
                .stopAndAdd(new ThreeBallShoot())

                .splineTo(new Vector2d(36, 34), Math.toRadians(-270))
                .stopAndAdd(new IntakeOn())
                .lineToY(65)
                .lineToY(35)

                .splineToConstantHeading(new Vector2d(-15, 15), Math.toRadians(180))
                .turnTo(Math.toRadians(-225))
                .stopAndAdd(new IntakeOff())
                .stopAndAdd(new ThreeBallShoot())

                .splineTo(new Vector2d(12, 34), Math.toRadians(-270))
                .stopAndAdd(new IntakeOn())
                .lineToY(65)
                .lineToY(35)

                .splineToConstantHeading(new Vector2d(-15, 15), Math.toRadians(180))
                .turnTo(Math.toRadians(-225))
                .stopAndAdd(new IntakeOff())
                .stopAndAdd(new ThreeBallShoot())

//                .splineTo(new Vector2d(-15, 34), Math.toRadians(-270))
//                .stopAndAdd(new IntakeOn())
//                .lineToY(60)
//                .lineToY(35)
//
//                .splineToConstantHeading(new Vector2d(-15, 15), Math.toRadians(180))
//                .turnTo(Math.toRadians(-225))
//                .stopAndAdd(new IntakeOff())


                .splineTo(new Vector2d(38, 33), Math.toRadians(-270))
                .build();


        waitForStart();


        Actions.runBlocking(new SequentialAction(
                BANEAuton
        ));
    }
}



