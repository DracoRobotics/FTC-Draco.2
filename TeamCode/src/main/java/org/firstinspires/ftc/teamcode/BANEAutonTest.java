package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantFunction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

@Disabled
@Autonomous(name = "BANE Autonomous with Shooter Actions")
public class BANEAutonTest extends LinearOpMode {

    // === Shooter Subsystem ===
    public static class Shooter {
        private final DcMotor flywheelLeft;
        private final DcMotor flywheelRight;
        private final DcMotor intakeMotor;
        private final Servo indexer;

        private static final double FLYWHEEL_POWER = 0.55;
        private static final double INDEXER_UP_POS = 1.0;
        private static final double INDEXER_DOWN_POS = 0.4;

        public Shooter(com.qualcomm.robotcore.hardware.HardwareMap hardwareMap) {
            flywheelLeft = hardwareMap.get(DcMotor.class, "flywheelLeft");
            flywheelRight = hardwareMap.get(DcMotor.class, "flywheelRight");
            intakeMotor = hardwareMap.get(DcMotor.class, "intake");
            indexer = hardwareMap.get(Servo.class, "indexer");

            flywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            flywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            indexer.setPosition(INDEXER_DOWN_POS);
        }

        // Spin up flywheel action (spin for 0.7 seconds)
        public Action spinUp() {
            return new Action() {
                private final ElapsedTime timer = new ElapsedTime();
                private boolean started = false;

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!started) {
                        flywheelLeft.setPower(FLYWHEEL_POWER);
                        flywheelRight.setPower(FLYWHEEL_POWER);
                        timer.reset();
                        started = true;
                    }

                    packet.put("SpinUpTimer", timer.seconds());

                    return timer.seconds() >= 0.7;
                }
            };
        }

        // Feed intake (run intake motor for 0.4 seconds)
        public Action feedIntake() {
            return new Action() {
                private final ElapsedTime timer = new ElapsedTime();
                private boolean started = false;

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!started) {
                        intakeMotor.setPower(-1.0);
                        timer.reset();
                        started = true;
                    }

                    packet.put("FeedIntakeTimer", timer.seconds());

                    if (timer.seconds() >= 0.4) {
                        intakeMotor.setPower(0);
                        return true;
                    }

                    return false;
                }
            };
        }

        // Indexer cycle: move up then down with delays
        public Action indexerCycle() {
            return new Action() {
                private final ElapsedTime timer = new ElapsedTime();
                private int stage = 0;

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    switch (stage) {
                        case 0:
                            indexer.setPosition(INDEXER_UP_POS);
                            timer.reset();
                            stage = 1;
                            break;

                        case 1:
                            if (timer.seconds() >= 0.8) {
                                indexer.setPosition(INDEXER_DOWN_POS);
                                timer.reset();
                                stage = 2;
                            }
                            break;

                        case 2:
                            if (timer.seconds() >= 0.8) {
                                return true;
                            }
                            break;
                    }
                    return false;
                }
            };
        }

        public Action stopAll() {
            return new Action() {
                private boolean done = false;

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!done) {
                        flywheelLeft.setPower(0);
                        flywheelRight.setPower(0);
                        intakeMotor.setPower(0);
                        indexer.setPosition(INDEXER_DOWN_POS);
                        done = true;
                    }
                    return true; // This action finishes immediately
                }
            };
        }


    }

    // === Composite Action to Shoot 3 Balls ===
    public static class ShootThreeBalls implements Action {
        private final Shooter shooter;
        private final SequentialAction sequence;

        public ShootThreeBalls(Shooter shooter) {
            this.shooter = shooter;

            List<Action> actions = new ArrayList<>();
            for (int i = 0; i < 3; i++) {
                actions.add(shooter.feedIntake());
                actions.add(shooter.indexerCycle());
            }
            sequence = new SequentialAction(actions);
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            boolean done = sequence.run(packet);
            if (done) {
                // Ensure intake is off after shooting
                shooter.intakeMotor.setPower(0);
            }
            return done;
        }
    }

    // === Hardware and Drive ===
    private Shooter shooter;
    private MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        shooter = new Shooter(hardwareMap);

        // Starting pose and drive initialization
        com.acmerobotics.roadrunner.Pose2d startPose = new com.acmerobotics.roadrunner.Pose2d(
                new Vector2d(65, -12),
                Math.toRadians(180)
        );
        drive = new MecanumDrive(hardwareMap, startPose);

        // Define a trajectory to drive to shooting position
        Action driveToShootPos = drive.actionBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-10, -10), Math.toRadians(180))
                .turnTo(Math.toRadians(230))
                .build();

        // Build the shooting routine as a sequential action
        SequentialAction shootingRoutine = new SequentialAction(
                shooter.spinUp(),
                new ShootThreeBalls(shooter),
                shooter.stopAll()
        );

        waitForStart();

        // Run drive then shooting sequentially
        Actions.runBlocking(new SequentialAction(
                driveToShootPos,
                shootingRoutine
        ));
    }
}
