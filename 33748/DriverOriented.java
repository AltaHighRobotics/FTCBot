package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "MAIN-DriverOriented")
public class DriverOriented extends LinearOpMode {

    private DcMotor motorFL, motorFR, motorBL, motorBR;
    private DcMotor intake, launcher;
    private Servo pushupservo;
    private HuskyLens husky;
    private IMU imu;

    private double initialHeading = 0.0;
    private boolean lastA = false;
    private boolean lastStart = false;
    private boolean lastDpadLeft = false;
    private long lastAPressTime = 0;

    private static final double LOW_RPM = 2800;
    private static final double HIGH_RPM = 5600;
    private static final double TPR = 28;
    private static final double KP = 0.0005;
    private static final double KI = 0.0001;
    private static final double KD = 0.0000005;
    
    private enum TeamMode { BLUE, RED }
    private TeamMode team = TeamMode.BLUE;
    private boolean lastDpadDown = false;


    @Override
    public void runOpMode() {

        motorFL = hardwareMap.get(DcMotor.class, "motor3");
        motorFR = hardwareMap.get(DcMotor.class, "motor2");
        motorBL = hardwareMap.get(DcMotor.class, "motor4");
        motorBR = hardwareMap.get(DcMotor.class, "motor1");

        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotor.class, "launcher");
        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pushupservo = hardwareMap.get(Servo.class, "pushupservo");
        husky = hardwareMap.get(HuskyLens.class, "husky");

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        ));

        initialHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        motorFL.setDirection(DcMotor.Direction.FORWARD);
        motorBL.setDirection(DcMotor.Direction.FORWARD);
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.REVERSE);
        
        PIDRPMController launcherPID = new PIDRPMController(TPR, KP, KI, KD);
        ElapsedTime timer = new ElapsedTime();

        boolean brakeMode = true;
        boolean lastBumperState = false;

        telemetry.update();
        waitForStart();
        timer.reset();

        while (opModeIsActive()) {

            boolean bumper = gamepad1.left_bumper;
            if (bumper && !lastBumperState) brakeMode = !brakeMode;
            lastBumperState = bumper;

            boolean startPressed = gamepad1.start;
            if (startPressed && !lastStart) {
                initialHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            }
            lastStart = startPressed;

            boolean dpadLeftPressed = gamepad1.dpad_left;
            if (dpadLeftPressed && !lastDpadLeft) {
                initialHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            }
            lastDpadLeft = dpadLeftPressed;

            double rawYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double botHeading = normalizeRadians(rawYaw - initialHeading);

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x;

            boolean ddown = gamepad1.dpad_down;
            if (ddown && !lastDpadDown) {
                if (team == TeamMode.BLUE) team = TeamMode.RED;
                else team = TeamMode.BLUE;
            }
            lastDpadDown = ddown;


            HuskyLens.Block[] blocks = husky.blocks();
            HuskyLens.Block tag = null;
            
            int targetId = (team == TeamMode.BLUE) ? 1 : 2;
            
            for (HuskyLens.Block block : blocks) {
                telemetry.addData("Block ID", block.id);
                telemetry.addData("Block X", block.x);
                if (block.id == targetId) {
                    tag = block;
                    break;
                }
            }

            if (tag != null) gamepad1.rumble(100);

            if (gamepad1.right_bumper && tag != null) {
                double cx = 180.0;
                double acceptableError = 15.0;
                double errorX = tag.x - cx;
                if (Math.abs(errorX) > acceptableError) turn = -(errorX / cx) * 0.9;
                else {
                    turn = 0;
                    telemetry.addData("Heading(deg)", Math.toDegrees(botHeading));
                    gamepad1.rumble(100);
                }
                y = 0;
                x = 0;
                motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } else {
                DcMotor.ZeroPowerBehavior behavior = brakeMode ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT;
                motorFL.setZeroPowerBehavior(behavior);
                motorFR.setZeroPowerBehavior(behavior);
                motorBL.setZeroPowerBehavior(behavior);
                motorBR.setZeroPowerBehavior(behavior);
            }

            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            double cosH = Math.cos(-botHeading);
            double sinH = Math.sin(-botHeading);
            double rotatedX = x * cosH - y * sinH;
            double rotatedY = x * sinH + y * cosH;

            double fl = rotatedY + rotatedX + turn;
            double fr = rotatedY - rotatedX - turn;
            double bl = rotatedY - rotatedX + turn;
            double br = rotatedY + rotatedX - turn;

            double max = Math.max(1.0, Math.max(Math.abs(fl),
                     Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
            motorFL.setPower(fl / max);
            motorFR.setPower(fr / max);
            motorBL.setPower(bl / max);
            motorBR.setPower(br / max);

            double intakePower = gamepad1.left_trigger;
            intake.setPower(intakePower);

            double trigger = gamepad1.right_trigger;
            double targetRPM = LOW_RPM + (HIGH_RPM - LOW_RPM) * trigger;
            for (HuskyLens.Block block : blocks) {
                if (block.id == targetId) {
                    targetRPM = 5800;
                    gamepad1.rumble(50);
                } else {
                    break;
                }
            }
            double launcherPower = launcherPID.update(launcher.getCurrentPosition(), targetRPM, timer.seconds());
            launcher.setPower(launcherPower);

            if (gamepad1.a) {
                long now = System.currentTimeMillis();
                if (now - lastAPressTime >= 2000) {
                    lastAPressTime = now;
                    pushupservo.setPosition(0.3);
                    sleep(500);
                    pushupservo.setPosition(0);
                    sleep(500);
                    pushupservo.setPosition(0.4);
                }
            } else {
                lastA = false;
            }

            if (gamepad1.y) {
                gamepad1.rumble(1000);
            }

            telemetry.addData("Motor mode", brakeMode ? "BRAKE" : "FLOAT");
            telemetry.addData("TAGGOAL", team == TeamMode.BLUE ? "BLUE, tag:20" : "RED, tag:24");
            telemetry.addData("Heading(deg)", Math.toDegrees(botHeading));
            telemetry.addData("TargetRPM", targetRPM);
            telemetry.addData("LauncherPower", launcherPower);
            telemetry.addData("Blocks", blocks.length);
            if (tag != null) {
                telemetry.addData("TagID", tag.id);
                telemetry.addData("TagX", tag.x);
                telemetry.addData("TagY", tag.y);
            }
            telemetry.update();

            sleep(20);
        }
    }

    private static double normalizeRadians(double angle) {
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle <= -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }
}
