package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "DriverOrientedDriving_FieldOriented")
public class DriverOriented extends LinearOpMode {

    // Drive motors
    private DcMotor motorFL, motorFR, motorBL, motorBR;

    // Other motors
    private DcMotor intake, launcher;

    // IMU
    private IMU imu;
    private double initialHeading = 0.0;

    // Launcher toggle state
    private double launcherPower = 0.0;
    private boolean lastY = false;

    // Button edge detectors
    private boolean lastB = false;
    private boolean lastA = false;
    private boolean lastStart = false;

    // Auto-fire state machine
    private enum AutoFireState { IDLE, SPINUP, KICK, COOLDOWN }
    private AutoFireState autoFireState = AutoFireState.IDLE;
    private long autoFireStateStartMs = 0L;

    // Push-up servo state
    private enum PushState { IDLE, EXTENDED, RETRACTING }
    private PushState pushState = PushState.IDLE;
    private long pushStateStartMs = 0L;

    // Constants
    private static final long SPINUP_MS = 1000;
    private static final long KICK_MS = 500;
    private static final long COOLDOWN_MS = 200;

    @Override
    public void runOpMode() {

        // ---------------- Hardware Map ----------------
        motorFL = hardwareMap.get(DcMotor.class, "motor3");
        motorFR = hardwareMap.get(DcMotor.class, "motor2");
        motorBL = hardwareMap.get(DcMotor.class, "motor4");
        motorBR = hardwareMap.get(DcMotor.class, "motor1");

        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotor.class, "launcher");

        // ---------------- Motor Directions ----------------
        motorFL.setDirection(DcMotor.Direction.FORWARD);
        motorBL.setDirection(DcMotor.Direction.FORWARD);
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.REVERSE);

        intake.setDirection(DcMotor.Direction.FORWARD);
        launcher.setDirection(DcMotor.Direction.FORWARD);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ---------------- IMU Initialization ----------------
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        ));

        // Capture initial heading at init (field forward reference)
        initialHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        boolean brakeMode = true;
        boolean lastBumperState = false;

        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            // ------------- Brake / float toggle ----------------
            boolean bumper = gamepad1.left_bumper;
            if (bumper && !lastBumperState) brakeMode = !brakeMode;
            lastBumperState = bumper;

            DcMotor.ZeroPowerBehavior behavior = brakeMode ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT;
            motorFL.setZeroPowerBehavior(behavior);
            motorFR.setZeroPowerBehavior(behavior);
            motorBL.setZeroPowerBehavior(behavior);
            motorBR.setZeroPowerBehavior(behavior);
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            // ------------- Start button resets field orientation ----------------
            boolean startPressed = gamepad1.start;
            if (startPressed && !lastStart) {
                initialHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            }
            lastStart = startPressed;

            // ---------------- Field-Oriented Drive ----------------
            double rawYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double botHeading = normalizeRadians(rawYaw - initialHeading);

            // Left stick for translation, right stick x for rotation
            double y = -gamepad1.left_stick_y; // forward/back
            double x = gamepad1.left_stick_x;  // left/right
            double turn = -gamepad1.right_stick_x; // rotation

            // Rotate joystick input by robot heading to get field-oriented motion
            double cosH = Math.cos(-botHeading);
            double sinH = Math.sin(-botHeading);
            double rotatedX = x * cosH - y * sinH;
            double rotatedY = x * sinH + y * cosH;

            // Mecanum wheel power
            double fl = rotatedY + rotatedX + turn;
            double fr = rotatedY - rotatedX - turn;
            double bl = rotatedY - rotatedX + turn;
            double br = rotatedY + rotatedX - turn;

            // Normalize
            double max = Math.max(1.0, Math.max(Math.abs(fl),
                     Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
            motorFL.setPower(fl / max);
            motorFR.setPower(fr / max);
            motorBL.setPower(bl / max);
            motorBR.setPower(br / max);

            // ---------------- Intake ----------------
            double intakePower = gamepad1.right_trigger;
            intake.setPower(intakePower);

            // ---------------- Launcher toggle ----------------
            if (gamepad1.y && !lastY) {
                launcherPower = (launcherPower == 0.0) ? 1.0 : 0.0;
                launcher.setPower(launcherPower);
            }
            lastY = gamepad1.y;

            // ---------------- Push-up servo (B) ----------------
            if (gamepad1.b && !lastB) {
                if (pushState == PushState.IDLE) {
                    pushState = PushState.EXTENDED;
                    pushStateStartMs = System.currentTimeMillis();
                } else if (pushState == PushState.EXTENDED) {
                    pushState = PushState.RETRACTING;
                    pushStateStartMs = System.currentTimeMillis();
                }
            }
            lastB = gamepad1.b;

            if (pushState == PushState.EXTENDED && System.currentTimeMillis() - pushStateStartMs > 600) {
                pushState = PushState.RETRACTING;
                pushStateStartMs = System.currentTimeMillis();
            } else if (pushState == PushState.RETRACTING && System.currentTimeMillis() - pushStateStartMs > 250) {
                pushState = PushState.IDLE;
            }

            // ---------------- Auto-fire single-shot (A) ----------------
            if (gamepad1.a && !lastA && autoFireState == AutoFireState.IDLE) {
                autoFireState = AutoFireState.SPINUP;
                autoFireStateStartMs = System.currentTimeMillis();
                launcher.setPower(0.3); // single-shot spin
            }
            lastA = gamepad1.a;

            long nowMs = System.currentTimeMillis();
            switch (autoFireState) {
                case SPINUP:
                    if (nowMs - autoFireStateStartMs >= SPINUP_MS) {
                        autoFireState = AutoFireState.KICK;
                        autoFireStateStartMs = nowMs;
                    }
                    break;
                case KICK:
                    if (nowMs - autoFireStateStartMs >= KICK_MS) {
                        launcher.setPower(0.0);
                        autoFireState = AutoFireState.COOLDOWN;
                        autoFireStateStartMs = nowMs;
                    }
                    break;
                case COOLDOWN:
                    if (nowMs - autoFireStateStartMs >= COOLDOWN_MS) {
                        autoFireState = AutoFireState.IDLE;
                    }
                    break;
                default: break;
            }

            // ---------------- Telemetry ----------------
            telemetry.addData("Mode", brakeMode ? "BRAKE" : "FLOAT");
            telemetry.addData("Heading(deg)", Math.toDegrees(botHeading));
            telemetry.addData("Launcher", launcherPower);
            telemetry.addData("AutoFireState", autoFireState.name());
            telemetry.update();

            sleep(20);
        }
    }

    // Normalize angle to [-PI, PI]
    private static double normalizeRadians(double angle) {
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle <= -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }
}
