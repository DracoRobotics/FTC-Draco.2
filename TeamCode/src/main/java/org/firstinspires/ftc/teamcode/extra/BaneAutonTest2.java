package org.firstinspires.ftc.teamcode.extra;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import androidx.annotation.NonNull;
@Disabled

@Autonomous(name = "ShooterFSMExample")
public class BaneAutonTest2 extends LinearOpMode {

    // Hardware
    private DcMotor intakeMotor;
    private DcMotor flywheelLeft;
    private DcMotor flywheelRight;
    private Servo indexer;

    // Constants
    private static final double FLYWHEEL_POWER = 0.55;
    private static final double INDEXER_UP_POS = 1.0;
    private static final double INDEXER_DOWN_POS = 0.4;


    enum State {
        SPIN_UP,
        FEED_INTAKE,
        INDEXER_UP,
        INDEXER_DOWN,
        DONE
    }
    // Shooter FSM as a Road Runner Action
    public class ShooterAction implements Action {



        private State state = State.SPIN_UP;
        private final ElapsedTime timer = new ElapsedTime();
        private int ballsShot = 0;
        private final int ballsToShoot = 3;

        public ShooterAction() {
            timer.reset();
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            switch(state) {
                case SPIN_UP:
                    flywheelLeft.setPower(FLYWHEEL_POWER);
                    flywheelRight.setPower(FLYWHEEL_POWER);
                    if (timer.seconds() >= 0.7) {
                        timer.reset();
                        state = State.FEED_INTAKE;
                    }
                    break;

                case FEED_INTAKE:
                    intakeMotor.setPower(-1.0);
                    if (timer.seconds() >= 0.4) {
                        intakeMotor.setPower(0);
                        indexer.setPosition(INDEXER_UP_POS);
                        timer.reset();
                        state = State.INDEXER_UP;
                    }
                    break;

                case INDEXER_UP:
                    if (timer.seconds() >= 0.8) {
                        indexer.setPosition(INDEXER_DOWN_POS);
                        timer.reset();
                        state = State.INDEXER_DOWN;
                    }
                    break;

                case INDEXER_DOWN:
                    if (timer.seconds() >= 0.8) {
                        ballsShot++;
                        if (ballsShot >= ballsToShoot) {
                            state = State.DONE;
                        } else {
                            timer.reset();
                            state = State.FEED_INTAKE;
                        }
                    }
                    break;

                case DONE:
                    flywheelLeft.setPower(0);
                    flywheelRight.setPower(0);
                    intakeMotor.setPower(0);
                    return true; // Action complete
            }
            return false; // Keep running
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        flywheelLeft = hardwareMap.get(DcMotor.class, "flywheelLeft");
        flywheelRight = hardwareMap.get(DcMotor.class, "flywheelRight");
        indexer = hardwareMap.get(Servo.class, "indexer");
        indexer.setPosition(INDEXER_DOWN_POS);

        waitForStart();

        // Create shooter action instance
        ShooterAction shooterAction = new ShooterAction();

        // Run shooter FSM repeatedly until done or opmode stopped
        while (opModeIsActive() && !shooterAction.run(null)) {
            // Optional: add telemetry for debugging
            telemetry.addData("Balls Shot", shooterAction.ballsShot);
            telemetry.addData("Shooter State", shooterAction.state);
            telemetry.update();

            idle(); // Let other system processes run
        }
    }
}
