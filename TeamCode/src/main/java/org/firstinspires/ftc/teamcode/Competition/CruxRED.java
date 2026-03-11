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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Configuration.MecanumDrive;


@Autonomous
public class CruxRED extends LinearOpMode {


    // ===== Hardware =====
    DcMotor intakeMotor;
    DcMotorEx flywheelLeft, flywheelRight;
    Servo indexer;
    private DcMotor turret;




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
        flywheelLeft.setVelocity(4000);
        flywheelRight.setVelocity(4000);


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
        sleep(3000);
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
        flywheelLeft = hardwareMap.get(DcMotorEx.class, "flywheelLeft");
        flywheelRight = hardwareMap.get(DcMotorEx.class, "flywheelRight");
        indexer = hardwareMap.get(Servo.class, "indexer");


        turret = hardwareMap.get(DcMotor.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        flywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Use Encoder mode for setVelocity
        flywheelLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);





        // PIDF Tuning for goBilda 6000 RPM Motors
// P = 12.0 (The 'kick' to reach speed)
// I = 3.0  (Helps stay at speed, but keep low to prevent windup)
// D = 0.0  (Usually not needed for flywheels)
// F = 12.8 (This is the 'cruise control' base power)
        // Recommended coefficients for a 6000 RPM Flywheel
        double F_COEFF = 6;
        double P_COEFF = 0.000000000001; // Start here and increase if recovery is slow
        double I_COEFF = 0;  // Keep very small
        double D_COEFF = 0.0;

        flywheelLeft.setVelocityPIDFCoefficients(P_COEFF, I_COEFF, D_COEFF, F_COEFF);
        flywheelRight.setVelocityPIDFCoefficients(P_COEFF, I_COEFF, D_COEFF, F_COEFF);



        Pose2d beginPose = new Pose2d(new Vector2d(70, 10), Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        indexer.setPosition(0.5);


        // ===== TRAJECTORY (UNCHANGED) =====
        Action BANEAuton = drive.actionBuilder(beginPose)

                .turnTo(Math.toRadians(-215))
                .stopAndAdd(new ThreeBallShoot())

                .splineTo(new Vector2d(35, 22), Math.toRadians(-270))
                .stopAndAdd(new IntakeOn())
                .lineToY(70)
                .lineToY(40)

                .splineToConstantHeading(new Vector2d(35, 22), Math.toRadians(-270))
                .turnTo(Math.toRadians(-215))
                .strafeTo(new Vector2d(70, 10))

                .build();


        waitForStart();


        Actions.runBlocking(new SequentialAction(
                BANEAuton
        ));
    }
}



