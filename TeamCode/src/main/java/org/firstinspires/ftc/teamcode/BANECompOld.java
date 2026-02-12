package org.firstinspires.ftc.teamcode;

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
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class BANECompOld extends OpMode {

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
    private IndexerState indexerState = IndexerState.CLOSED;

    private static final double INDEXER_OPEN_POS = 0.8;
    private static final double INDEXER_CLOSED_POS = 0.5;
    private static final double INDEXER_MOVE_TIME = 0.25;

    // ---------------- SHOOTER ----------------
    private enum ShooterState { IDLE, SPINNING_UP, OPEN_GATE, FEEDING, CLOSE_GATE }
    private ShooterState shooterState = ShooterState.IDLE;

    private static final double FEED_TIME = 3.0;
    private boolean shootButtonLast = false;
    double flywheelRPM = 0; // Global scope for telemetry access
    double flywheelSpeedUpTime = 0.6;

    // ---------------- TURRET PID ----------------
    private static final double TICKS_PER_DEGREE = 9.744444;
    private static final double MAX_TURN_POWER = 1.0;
    private double Kp = 0.0015;
    private double Ki = 0.0000001;
    private double Kd = 0.00;
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

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");

        flywheelLeft = hardwareMap.get(DcMotorEx.class, "flywheelLeft");
        flywheelRight = hardwareMap.get(DcMotorEx.class, "flywheelRight");

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

        // Reverse one if motors face each other
        // flywheelRight.setDirection(DcMotor.Direction.REVERSE);

        indexer = hardwareMap.get(Servo.class, "indexer");
        indexer.setPosition(INDEXER_CLOSED_POS);

        lastTimeMs = System.currentTimeMillis();
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
        handleShootButton();
        updateShooter();
        updateIndexer();
        updateTurret();
        updateDrive();
        updateManualIntake();
        updateTelemetry();





    }

    public void handleShootButton() { 
        boolean shoot = gamepad1.cross;
        if (shoot && !shootButtonLast && shooterState == ShooterState.IDLE) {
            shooterState = ShooterState.SPINNING_UP;
            shooterTimer.reset();
        }
        shootButtonLast = shoot;
    }

    public void updateShooter() {
        // 1. Calculate Target RPM from Vision
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double ty = result.getTy();
            double totalAngle = Math.toRadians(45 + ty);
            double X = (42 - 16) / Math.tan(totalAngle);
            flywheelRPM = (30 * X) / (0.205  * Math.PI * Math.cos(Math.toRadians(45)));
        }

        // 2. Continuous Velocity Update
        double targetTicksPerSec = flywheelRPM;  //(flywheelRPM / 60.0) * 28;
        double currentRPM = (flywheelLeft.getVelocity());

        if (shooterState != ShooterState.IDLE) {
            flywheelLeft.setVelocity(targetTicksPerSec);
            flywheelRight.setVelocity(targetTicksPerSec);
        }

        // 3. Sequential Shooter State Machine
        switch (shooterState) {
            case IDLE:
                flywheelLeft.setVelocity(0);
                flywheelRight.setVelocity(0);
                intakeMotor.setPower(0);
                break;

            case SPINNING_UP:
                // --- TOLERANCE CHECK ---
                // Only proceed if we are within 150 RPM of the target
                // and we've waited at least 0.4s to prevent false positives
                double error = Math.abs(currentRPM - flywheelRPM);

                if (shooterTimer.seconds() > flywheelSpeedUpTime) {
                    openGate();
                    shooterTimer.reset();
                    shooterState = ShooterState.FEEDING;
                }
                break;

            case FEEDING:
                intakeMotor.setPower(-1.0);
                if (shooterTimer.seconds() > FEED_TIME) {
                    intakeMotor.setPower(0);
                    shooterState = ShooterState.CLOSE_GATE;
                }
                break;

            case CLOSE_GATE:
                closeGate();
                shooterState = ShooterState.IDLE;
                break;
        }
    }


    private void updateIndexer() {
        switch (indexerState) {
            case CLOSED: indexer.setPosition(INDEXER_CLOSED_POS); break;
            case OPENING:
                indexer.setPosition(INDEXER_OPEN_POS);
                if (indexerTimer.seconds() > INDEXER_MOVE_TIME) indexerState = IndexerState.OPEN;
                break;
            case CLOSING:
                indexer.setPosition(INDEXER_CLOSED_POS);
                if (indexerTimer.seconds() > INDEXER_MOVE_TIME) indexerState = IndexerState.CLOSED;
                break;
            case OPEN: break;
        }
    }

    private void updateTurret() {
        if (gamepad1.dpad_left) headingLockEnabled = true;
        if (gamepad1.dpad_right) headingLockEnabled = false;

        limelightResult = limelight.getLatestResult();
        if (!headingLockEnabled || limelightResult == null || !limelightResult.isValid()) {
            turret.setPower(0);
            integralSum = 0;
            return;
        }

        double rawTx = limelightResult.getTx();
        double tx = 0.25 * rawTx + 0.75 * lastFilteredTx;
        lastFilteredTx = tx;

        double errorTicks = tx * TICKS_PER_DEGREE;
        long now = System.currentTimeMillis();
        double dt = Math.max((now - lastTimeMs) / 1000.0, 0.02);
        lastTimeMs = now;

        integralSum += errorTicks * dt;
        double derivative = (errorTicks - lastError) / dt;

        double power = Kp * errorTicks + Ki * integralSum + Kd * derivative;
        turret.setPower(Range.clip(power, -MAX_TURN_POWER, MAX_TURN_POWER));
        lastError = errorTicks;
    }

    private void updateDrive() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        double lf = y + x + rx;
        double rf = y - x - rx;
        double lb = y - x + rx;
        double rb = y + x - rx;
        double max = Math.max(Math.max(Math.abs(lf), Math.abs(rf)), Math.max(Math.abs(lb), Math.abs(rb)));
        if (max > 1.0) { lf /= max; rf /= max; lb /= max; rb /= max; }
        leftFrontDrive.setPower(lf);
        rightFrontDrive.setPower(rf);
        leftBackDrive.setPower(lb);
        rightBackDrive.setPower(rb);
    }

    private void updateManualIntake() {
        if (shooterState != ShooterState.IDLE) return;
        if (gamepad1.right_bumper) intakeMotor.setPower(-1);
        else if (gamepad1.left_bumper) intakeMotor.setPower(1);
        else intakeMotor.setPower(0);
    }

    private void updateTelemetry() {
        double currentVel = flywheelLeft.getVelocity();
        double targetVel = flywheelRPM;
        boolean atSpeed = Math.abs(currentVel - targetVel) < 50 && targetVel > 0;
        boolean visionLock = (limelight.getLatestResult() != null && limelight.getLatestResult().isValid());

        telemetry.addData("READY", (visionLock && atSpeed) ? "YES" : "WAITING");
        telemetry.addData("Target RPM", "%.2f", flywheelRPM);
        telemetry.addData("Actual RPM", "%.2f", (currentVel * 60.0) / 28.0);
        telemetry.addData("Shooter State", shooterState);
        telemetry.update();
    }

    private void openGate() { if (indexerState == IndexerState.CLOSED) { indexerTimer.reset(); indexerState = IndexerState.OPENING; } }
    private void closeGate() { if (indexerState == IndexerState.OPEN) { indexerTimer.reset(); indexerState = IndexerState.CLOSING; } }
}