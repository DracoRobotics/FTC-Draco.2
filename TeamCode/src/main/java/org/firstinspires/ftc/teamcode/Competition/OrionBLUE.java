package org.firstinspires.ftc.teamcode.Competition;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantFunction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.Configuration.MecanumDrive;

@Autonomous
public class OrionBLUE extends LinearOpMode {

    // ===== Hardware =====
    DcMotor intakeMotor;
    DcMotorEx flywheelLeft, flywheelRight;
    Servo indexer;

    // ===== Turret =====
    DcMotor turret;
    Limelight3A limelight;

    private static final double TICKS_PER_DEGREE = 9.744444;
    private static final double MAX_TURN_POWER   = 1.0;
    private static final double TURRET_LOCK_TOLERANCE_TICKS = 10.0; // ~1 degree

    private double Kp = 0.0015;
    private double Ki = 0.0000001;
    private double Kd = 0.00;
    private double integralSum  = 0.0;
    private double lastError    = 0.0;
    private long   lastTimeMs   = 0;
    private double lastFilteredTx = 0.0;

    // ===== Turret Thread =====
    private volatile boolean turretRunning = false;
    private Thread turretThread;

    // ===== Shooter =====
    private static final double LOCK_TIMEOUT_SEC = 2.0; // max wait for vision lock

    // ------------------------------------------------
    // TURRET HELPERS
    // ------------------------------------------------

    /** Single PID tick — call this in a loop from the turret thread. */
    private void turretStep() {
        LLResult result = limelight.getLatestResult();

        double errorTicks;

        if (result != null && result.isValid()) {
            // Vision lock: drive tx to zero
            double rawTx = result.getTx();
            double tx = 0.25 * rawTx + 0.75 * lastFilteredTx;
            lastFilteredTx = tx;
            errorTicks = tx * TICKS_PER_DEGREE;
        } else {
            // No target: return to encoder centre (0)
            errorTicks = turret.getCurrentPosition();
            if (Math.abs(errorTicks) < 5) {
                turret.setPower(0);
                integralSum = 0;
                lastError   = 0;
                return;
            }
        }

        long   now  = System.currentTimeMillis();
        double dt   = Math.max((now - lastTimeMs) / 1000.0, 0.02);
        lastTimeMs  = now;

        integralSum += errorTicks * dt;
        double derivative = (errorTicks - lastError) / dt;
        double power = Kp * errorTicks + Ki * integralSum + Kd * derivative;

        turret.setPower(Range.clip(power, -MAX_TURN_POWER, MAX_TURN_POWER));
        lastError = errorTicks;
    }

    /** Returns true when the turret has a valid vision lock AND is aimed within tolerance. */
    private boolean isTurretLocked() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return false;
        double errorTicks = result.getTx() * TICKS_PER_DEGREE;
        return Math.abs(errorTicks) < TURRET_LOCK_TOLERANCE_TICKS;
    }

    /** Blocks until turret is locked or timeout expires. */
    private void waitForTurretLock() {
        double startSec = System.currentTimeMillis() / 1000.0;
        while (opModeIsActive()) {
            if (isTurretLocked()) return;
            if ((System.currentTimeMillis() / 1000.0 - startSec) > LOCK_TIMEOUT_SEC) return;
            sleep(20);
        }
    }

    // ------------------------------------------------
    // FLYWHEEL / INTAKE / INDEXER
    // ------------------------------------------------

    public void intakeRun()  { intakeMotor.setPower(-1); }
    public void intakeStop() { intakeMotor.setPower(0);  }
    public void outtake()    { intakeMotor.setPower(1);  }

    public void spinFlywheel() {
        flywheelLeft.setVelocity(3300);
        flywheelRight.setVelocity(3300);
    }
    public void stopFlywheel() {
        flywheelLeft.setPower(0);
        flywheelRight.setPower(0);
    }

    public void indexerOpen()  { indexer.setPosition(1); }
    public void indexerClose() { indexer.setPosition(0.8); }

    /**
     * Waits for turret lock, then shoots three balls.
     * Safe to call from InstantFunction because the turret thread
     * keeps tracking in the background.
     */
    public void threeBallShoot() {
        waitForTurretLock();        // block until aimed (or timeout)
        indexerOpen();
        spinFlywheel();
        sleep(900);
        intakeRun();
        sleep(3000);
        intakeStop();
        indexerClose();
        stopFlywheel();
    }

    // ------------------------------------------------
    // ROAD RUNNER INSTANT ACTIONS
    // ------------------------------------------------

    public class ThreeBallShoot implements InstantFunction {
        @Override public void run() { threeBallShoot(); }
    }
    public class IntakeOn implements InstantFunction {
        @Override public void run() { intakeMotor.setPower(-1); }
    }
    public class IntakeOff implements InstantFunction {
        @Override public void run() { intakeMotor.setPower(0); }
    }

    // ------------------------------------------------
    // MAIN
    // ------------------------------------------------

    @Override
    public void runOpMode() {

        // ----- Hardware init -----
        intakeMotor  = hardwareMap.get(DcMotor.class,   "intake");
        flywheelLeft  = hardwareMap.get(DcMotorEx.class, "flywheelLeft");
        flywheelRight = hardwareMap.get(DcMotorEx.class, "flywheelRight");
        indexer       = hardwareMap.get(Servo.class,     "indexer");

        turret   = hardwareMap.get(DcMotor.class,     "turret");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Turret setup (mirrors TeleOp)
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Limelight setup
        limelight.pipelineSwitch(0);
        limelight.start();

        // Flywheel setup
        flywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double F_COEFF = 6, P_COEFF = 0.000000000001, I_COEFF = 0, D_COEFF = 0.0;
        flywheelLeft.setVelocityPIDFCoefficients(P_COEFF, I_COEFF, D_COEFF, F_COEFF);
        flywheelRight.setVelocityPIDFCoefficients(P_COEFF, I_COEFF, D_COEFF, F_COEFF);

        indexer.setPosition(0.5);

        // ----- Road Runner -----
        Pose2d beginPose = new Pose2d(new Vector2d(65, -12), Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        Action BANEAuton = drive.actionBuilder(beginPose)

                .lineToX(-15)
                .turnTo(Math.toRadians(225))
                .stopAndAdd(new ThreeBallShoot())

                .turnTo(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-12, -31), Math.toRadians(270))
                .lineToY(-70)
                .lineToY(-31)
                .splineToConstantHeading(new Vector2d(-55, -12), Math.toRadians(270))
                .stopAndAdd(new ThreeBallShoot())

                .splineToConstantHeading(new Vector2d(12, -31), Math.toRadians(270))
                .lineToY(-70)
                .lineToY(-31)
                .splineToConstantHeading(new Vector2d(-55, -12), Math.toRadians(270))
                .stopAndAdd(new ThreeBallShoot())



                .splineTo(new Vector2d(38, -33), Math.toRadians(270))
                .build();

        waitForStart();
        lastTimeMs = System.currentTimeMillis();

        // ----- Start turret background thread -----
        turretRunning = true;
        turretThread = new Thread(() -> {
            while (turretRunning && opModeIsActive()) {
                turretStep();
                try { Thread.sleep(20); } catch (InterruptedException e) { Thread.currentThread().interrupt(); }
            }
            turret.setPower(0); // safe stop when thread exits
        });
        turretThread.start();

        // ----- Run autonomous -----
        try {
            Actions.runBlocking(new SequentialAction(BANEAuton));
        } finally {
            // Always clean up the turret thread, even if auton is stopped early
            turretRunning = false;
            try { turretThread.join(500); } catch (InterruptedException ignored) {}
            turret.setPower(0);
            limelight.stop();
        }
    }
}