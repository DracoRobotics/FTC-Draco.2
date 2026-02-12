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
public class BANECompNew extends OpMode {


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
    private enum ShooterState { IDLE, SPINNING_UP, FEEDING, CLOSE_GATE }
    private ShooterState shooterState = ShooterState.IDLE;


    private static final double FEED_TIME = 3.0;


    // Flywheel constants
    private static final double TICKS_PER_REV = 28.0;
    private static final double RPM_TOLERANCE = 75.0;
    private static final double MIN_SPINUP_TIME = 0.3;


    private boolean shootButtonLast = false;


    // ---------------- TURRET PID ----------------
    private static final double TICKS_PER_DEGREE = 9.744444;
    private static final double MAX_TURN_POWER = 1.0;


    private double Kp = 0.000059;
    private double Ki = 0.000055;
    private double Kd = 0.0002;


    private double integralSum = 0.0;
    private double lastError = 0.0;
    private long lastTimeMs = 0;


    private double lastFilteredTx = 0.0;
    private boolean headingLockEnabled = false;


    // ---------------- INIT ----------------
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


        intakeMotor = hardwareMap.get(DcMotor.class, "intake");


        flywheelLeft  = hardwareMap.get(DcMotorEx.class, "flywheelLeft");
        flywheelRight = hardwareMap.get(DcMotorEx.class, "flywheelRight");


        // 🔥 CRITICAL CHANGE
        flywheelLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        indexer = hardwareMap.get(Servo.class, "indexer");
        indexer.setPosition(INDEXER_CLOSED_POS);


        lastTimeMs = System.currentTimeMillis();
    }


    @Override
    public void start() {
        limelight.start();
    }


    // ---------------- LOOP ----------------
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


    // ================= SHOOTER =================


    public void handleShootButton() {
        boolean shoot = gamepad1.cross;
        if (shoot && !shootButtonLast && shooterState == ShooterState.IDLE) {
            shooterState = ShooterState.SPINNING_UP;
            shooterTimer.reset();
        }
        shootButtonLast = shoot;
    }


    public void updateShooter() {


        // ----- LIMELIGHT DISTANCE → RPM -----
        double CAMERA_HEIGHT = 16;
        double TARGET_HEIGHT = 42;
        double CAMERA_ANGLE_DEG = 45;


        LLResult result = limelight.getLatestResult();
        double X = 0;


        if (result != null && result.isValid()) {
            double ty = result.getTy();
            double totalAngle = Math.toRadians(CAMERA_ANGLE_DEG + ty);
            X = (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(totalAngle);
        }


        double theoreticalRPM = (30 * X) / (0.205 * Math.PI * Math.cos(Math.toRadians(45)));
        double targetRPM = theoreticalRPM * 0.85;


        double currentRPM =
                flywheelLeft.getVelocity() * 60.0 / TICKS_PER_REV;


        switch (shooterState) {


            case IDLE:
                flywheelLeft.setVelocity(0);
                flywheelRight.setVelocity(0);
                intakeMotor.setPower(0);
                break;


            case SPINNING_UP:
                setFlywheelRPM(6000);


                if (Math.abs(currentRPM - targetRPM) < RPM_TOLERANCE &&
                        shooterTimer.seconds() > MIN_SPINUP_TIME) {
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


    private void setFlywheelRPM(double rpm) {
        double ticksPerSec = rpm * TICKS_PER_REV / 60.0;
        flywheelLeft.setVelocity(ticksPerSec);
        flywheelRight.setVelocity(ticksPerSec);
    }


    // ================= INDEXER =================


    private void updateIndexer() {
        switch (indexerState) {
            case CLOSED:
                indexer.setPosition(INDEXER_CLOSED_POS);
                break;


            case OPENING:
                indexer.setPosition(INDEXER_OPEN_POS);
                if (indexerTimer.seconds() > INDEXER_MOVE_TIME)
                    indexerState = IndexerState.OPEN;
                break;


            case CLOSING:
                indexer.setPosition(INDEXER_CLOSED_POS);
                if (indexerTimer.seconds() > INDEXER_MOVE_TIME)
                    indexerState = IndexerState.CLOSED;
                break;


            case OPEN:
                break;
        }
    }


    // ================= TURRET =================


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
        double dt = (now - lastTimeMs) / 1000.0;
        if (dt <= 0) dt = 0.02;
        lastTimeMs = now;


        integralSum += errorTicks * dt;
        double derivative = (errorTicks - lastError) / dt;


        double power = Kp * errorTicks + Ki * integralSum + Kd * derivative;
        turret.setPower(Range.clip(power, -MAX_TURN_POWER, MAX_TURN_POWER));


        lastError = errorTicks;
    }


    // ================= DRIVE =================


    private void updateDrive() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;


        double lf = y + x + rx;
        double rf = y - x - rx;
        double lb = y - x + rx;
        double rb = y + x - rx;


        double max = Math.max(Math.max(Math.abs(lf), Math.abs(rf)),
                Math.max(Math.abs(lb), Math.abs(rb)));


        if (max > 1.0) {
            lf /= max; rf /= max; lb /= max; rb /= max;
        }


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
        telemetry.addData("Shooter State", shooterState);
        telemetry.addData("Indexer State", indexerState);
        telemetry.addData("Flywheel RPM",
                flywheelLeft.getVelocity() * 60.0 / TICKS_PER_REV);
        telemetry.addData("RPM", flywheelLeft.getVelocity());
        telemetry.update();
    }


    // ---------------- GATE ----------------


    private void openGate() {
        if (indexerState == IndexerState.CLOSED) {
            indexerTimer.reset();
            indexerState = IndexerState.OPENING;
        }
    }


    private void closeGate() {
        if (indexerState == IndexerState.OPEN) {
            indexerTimer.reset();
            indexerState = IndexerState.CLOSING;
        }
    }
}





