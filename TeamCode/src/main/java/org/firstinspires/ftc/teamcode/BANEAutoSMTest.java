package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled

@Autonomous
public class BANEAutoSMTest extends LinearOpMode {

    // ================= HARDWARE =================
    DcMotor intakeMotor;
    DcMotor flywheelLeft, flywheelRight;
    Servo indexer;

    // ================= CONSTANTS =================
    static final double FLYWHEEL_POWER = 0.55;

    static final double INDEXER_UP = 1.0;
    static final double INDEXER_DOWN = 0.4;

    static final double SPINUP_TIME = 0.7;
    static final double INTAKE_TIME = 0.05;
    static final double INDEXER_UP_TIME = 0.8;
    static final double INDEXER_DOWN_TIME = 0.8;

    static final int BALLS_TO_SHOOT = 3;

    // ================= FSM =================
    enum ShooterState {
        SPIN_UP,
        FEED_INTAKE,
        INDEXER_UP,
        INDEXER_DOWN,
        DONE
    }

    class ShooterAction implements Action {

        private ShooterState state = ShooterState.SPIN_UP;
        private final ElapsedTime timer = new ElapsedTime();
        private int ballsShot = 0;

        ShooterAction() {
            timer.reset();
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            switch (state) {

                case SPIN_UP:
                    flywheelLeft.setPower(FLYWHEEL_POWER);
                    flywheelRight.setPower(FLYWHEEL_POWER);

                    if (timer.seconds() >= SPINUP_TIME) {
                        timer.reset();
                        state = ShooterState.FEED_INTAKE;
                    }
                    break;

                case FEED_INTAKE:
                    intakeMotor.setPower(-1);

                    if (timer.seconds() >= INTAKE_TIME) {
                        intakeMotor.setPower(0);
                        indexer.setPosition(INDEXER_UP);
                        timer.reset();
                        state = ShooterState.INDEXER_UP;
                    }
                    break;

                case INDEXER_UP:
                    if (timer.seconds() >= INDEXER_UP_TIME) {
                        indexer.setPosition(INDEXER_DOWN);
                        timer.reset();
                        state = ShooterState.INDEXER_DOWN;
                    }
                    break;

                case INDEXER_DOWN:
                    if (timer.seconds() >= INDEXER_DOWN_TIME) {
                        ballsShot++;

                        if (ballsShot >= BALLS_TO_SHOOT) {
                            state = ShooterState.DONE;
                        } else {
                            timer.reset();
                            state = ShooterState.FEED_INTAKE;
                        }
                    }
                    break;

                case DONE:
                    intakeMotor.setPower(0);
                    flywheelLeft.setPower(0);
                    flywheelRight.setPower(0);
                    return true; // ACTION COMPLETE
            }

            return false; // KEEP RUNNING
        }
    }

    // ================= OPMODE =================
    @Override
    public void runOpMode() {

        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        flywheelLeft = hardwareMap.get(DcMotor.class, "flywheelLeft");
        flywheelRight = hardwareMap.get(DcMotor.class, "flywheelRight");
        indexer = hardwareMap.get(Servo.class, "indexer");

        indexer.setPosition(INDEXER_DOWN);

        flywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        Pose2d beginPose = new Pose2d(new Vector2d(65, -12), Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        ShooterAction shooterAction = new ShooterAction();

        // ===== TEST ACTION ONLY =====
        Action testAction = drive.actionBuilder(beginPose)
                .stopAndAdd(new ShooterAction())
                .build();

        waitForStart();

        Actions.runBlocking(shooterAction);
    }
}
