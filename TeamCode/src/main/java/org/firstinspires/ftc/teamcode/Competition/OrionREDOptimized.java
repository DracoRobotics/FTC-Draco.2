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
public class OrionREDOptimized extends LinearOpMode {

    // ===== Hardware =====
    DcMotor intakeMotor;
    DcMotor flywheelLeft, flywheelRight;
    Servo indexer;

    // ===== Intake =====
    public void intakeRun() { intakeMotor.setPower(-1); }
    public void intakeStop() { intakeMotor.setPower(0); }
    public void Outtake() { intakeMotor.setPower(1); }

    // ===== Flywheel =====
    public void SpinFlywheel() {
        flywheelLeft.setPower(0.6);
        flywheelRight.setPower(0.6);
    }
    public void StopFlywheel() {
        flywheelLeft.setPower(0);
        flywheelRight.setPower(0);
    }

    // ===== Indexer =====
    public void IndexerOpen() { indexer.setPosition(0.8); }
    public void IndexerClose() { indexer.setPosition(0.5); }

    // ===== Improved 3-Ball Shoot with Bursts =====
    public void ThreeBallShootSequence() {
        IndexerOpen();
        SpinFlywheel();
        sleep(600); // spin stabilization

        // Feed each ball individually
        for (int i = 0; i < 3; i++) {
            intakeRun();
            sleep(500);  // push one ball
            intakeStop();
            sleep(200);  // allow ball to fully seat
        }

        IndexerClose();
        StopFlywheel();
    }

    // ===== Road Runner Instant Actions =====
    public class ShootCycle implements InstantFunction {
        @Override
        public void run() { ThreeBallShootSequence(); }
    }

    public class IntakeOn implements InstantFunction {
        @Override
        public void run() { intakeRun(); }
    }

    public class IntakeOff implements InstantFunction {
        @Override
        public void run() { intakeStop(); }
    }

    @Override
    public void runOpMode() {
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        flywheelLeft = hardwareMap.get(DcMotor.class, "flywheelLeft");
        flywheelRight = hardwareMap.get(DcMotor.class, "flywheelRight");
        indexer = hardwareMap.get(Servo.class, "indexer");

        flywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        Pose2d beginPose = new Pose2d(new Vector2d(65, 12), Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        Action BANEAuton = drive.actionBuilder(beginPose)

                .splineToConstantHeading(new Vector2d(-16, 16), Math.toRadians(180))
                .turnTo(Math.toRadians(-225))
                .stopAndAdd(new ShootCycle())

                .splineTo(new Vector2d(36, 34), Math.toRadians(-270))
                .stopAndAdd(new IntakeOn())
                .lineToY(65)
                .lineToY(35)

                .splineToConstantHeading(new Vector2d(-16, 16), Math.toRadians(180))
                .turnTo(Math.toRadians(-225))
                .stopAndAdd(new IntakeOff())
                .stopAndAdd(new ShootCycle())

                .splineTo(new Vector2d(12, 34), Math.toRadians(-270))
                .stopAndAdd(new IntakeOn())
                .lineToY(65)
                .lineToY(35)

                .splineToConstantHeading(new Vector2d(-16, 16), Math.toRadians(180))
                .turnTo(Math.toRadians(-225))
                .stopAndAdd(new IntakeOff())
                .stopAndAdd(new ShootCycle())

                .splineTo(new Vector2d(-7, 34), Math.toRadians(-270))
                .stopAndAdd(new IntakeOn())
                .lineToY(65)
                .lineToY(35)

                .splineToConstantHeading(new Vector2d(-16, 16), Math.toRadians(180))
                .turnTo(Math.toRadians(-225))
                .stopAndAdd(new IntakeOff())
                .stopAndAdd(new ShootCycle())

                .splineTo(new Vector2d(38, 33), Math.toRadians(-270))
                .build();

        waitForStart();
        Actions.runBlocking(new SequentialAction(BANEAuton));
    }
}
