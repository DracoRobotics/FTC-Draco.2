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
public class OptimizedTestOrionBLUE extends LinearOpMode {

    // ===== Hardware =====
    DcMotor intakeMotor;
    DcMotorEx flywheelLeft, flywheelRight;
    Servo indexer;

    // ===== Intake =====
    public void intakeRun() { intakeMotor.setPower(-1); }
    public void intakeStop() { intakeMotor.setPower(0); }
    public void Outtake() { intakeMotor.setPower(1); }

    // ===== Flywheel =====
    public void SpinFlywheel() {
        flywheelLeft.setVelocity(3000);
        flywheelRight.setVelocity(3000);
    }

    public void StopFlywheel() {
        flywheelLeft.setPower(0);
        flywheelRight.setPower(0);
    }

    // ===== Indexer =====
    public void IndexerOpen() { indexer.setPosition(0.8); }
    public void IndexerClose() { indexer.setPosition(0.5); }

    // ===== Improved 3-Ball Shoot =====
    public void ThreeBallShootSequence() {
        IndexerOpen();
        sleep(600);  // reduced spin stabilization

        // Loop to feed 3 balls reliably
        for (int i = 0; i < 3; i++) {
            intakeRun();
            sleep(500); // run intake in short bursts
            intakeStop();
            sleep(200); // give balls time to fully pass
        }

        IndexerClose();
        StopFlywheel();
    }

    // ===== Road Runner Instant Actions =====
    public class SpinUp implements InstantFunction {
        @Override
        public void run() { SpinFlywheel(); }
    }

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
        flywheelLeft = hardwareMap.get(DcMotorEx.class, "flywheelLeft");
        flywheelRight = hardwareMap.get(DcMotorEx.class, "flywheelRight");
        indexer = hardwareMap.get(Servo.class, "indexer");

        flywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        flywheelLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // PIDF (unchanged)
        double F_COEFF = 6;
        double P_COEFF = 0.000000000001;
        double I_COEFF = 0;
        double D_COEFF = 0.0;
                
        flywheelLeft.setVelocityPIDFCoefficients(P_COEFF, I_COEFF, D_COEFF, F_COEFF);
        flywheelRight.setVelocityPIDFCoefficients(P_COEFF, I_COEFF, D_COEFF, F_COEFF);

        Pose2d beginPose = new Pose2d(new Vector2d(65, -12), Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        // ===== TRAJECTORY (UNCHANGED PATH) =====
        Action BANEAuton = drive.actionBuilder(beginPose)

                .splineToConstantHeading(new Vector2d(-16, -16), Math.toRadians(180))
                .stopAndAdd(new SpinUp())
                .turnTo(Math.toRadians(225))
                .stopAndAdd(new ShootCycle())

                .splineTo(new Vector2d(39, -34), Math.toRadians(270))
                .stopAndAdd(new IntakeOn())
                .lineToY(-70)
                .lineToY(-40)

                .splineToConstantHeading(new Vector2d(-16, -16), Math.toRadians(180))
                .stopAndAdd(new SpinUp())
                .turnTo(Math.toRadians(225))
                .stopAndAdd(new IntakeOff())
                .stopAndAdd(new ShootCycle())

                .splineTo(new Vector2d(15, -34), Math.toRadians(270))
                .stopAndAdd(new IntakeOn())
                .lineToY(-70)
                .lineToY(-40)

                .splineToConstantHeading(new Vector2d(-16, -16), Math.toRadians(180))
                .stopAndAdd(new SpinUp())
                .turnTo(Math.toRadians(225))
                .stopAndAdd(new IntakeOff())
                .stopAndAdd(new ShootCycle())

                .splineTo(new Vector2d(-7, -34), Math.toRadians(270))
                .stopAndAdd(new IntakeOn())
                .lineToY(-70)
                .lineToY(-40)

                .splineToConstantHeading(new Vector2d(-16, -16), Math.toRadians(180))
                .stopAndAdd(new SpinUp())
                .turnTo(Math.toRadians(225))
                .stopAndAdd(new IntakeOff())
                .stopAndAdd(new ShootCycle())

                .splineTo(new Vector2d(38, -33), Math.toRadians(270))

                .build();

        waitForStart();

        Actions.runBlocking(new SequentialAction(
                BANEAuton
        ));
    }
}
