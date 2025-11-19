package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "MainDrive")
public class MainDrive extends LinearOpMode {
   
    private volatile boolean aActionRunning = false;
    private volatile boolean bActionRunning = false;
    private volatile boolean xActionRunning = false;
   
    private DcMotor motorFL, motorFR, motorBL, motorBR;
    private DcMotor intake, rotaryspin, launcher;
    private IMU imu;
    private Servo pushup, rotarykick;
    
    private double initialHeading = 0;
    private double launcherPower = 0.0;
    private int x = 0;
    private boolean lastY = false;
    private boolean lastB = false;
    private boolean lastA = false;

    private static final double TICKS_PER_REV = 1425.1;
    private static final double DEGREES_PER_TICK = 360.0 / TICKS_PER_REV;
    private static final double TARGET_DEGREES = 119.0;
    private static final int TARGET_TICKS = (int) (TARGET_DEGREES / DEGREES_PER_TICK);

    @Override
    public void runOpMode() {
        
        motorFL = hardwareMap.get(DcMotor.class, "motor2");
        motorBR = hardwareMap.get(DcMotor.class, "motor1");
        motorFR = hardwareMap.get(DcMotor.class, "motor3");
        motorBL = hardwareMap.get(DcMotor.class, "motor4");

        intake = hardwareMap.get(DcMotor.class, "intake");
        rotaryspin = hardwareMap.get(DcMotor.class, "rotaryspin");
        launcher = hardwareMap.get(DcMotor.class, "launcher");

        pushup = hardwareMap.get(Servo.class, "pushup");
        rotarykick = hardwareMap.get(Servo.class, "rotarykick");

        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.FORWARD);
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.REVERSE);

        intake.setDirection(DcMotor.Direction.FORWARD);
        rotaryspin.setDirection(DcMotor.Direction.FORWARD);
        launcher.setDirection(DcMotor.Direction.FORWARD);

        rotaryspin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotaryspin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotaryspin.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pushup.setPosition(0);
        rotarykick.setPosition(0);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                );
        IMU.Parameters parameters = new IMU.Parameters(orientation);
        imu.initialize(parameters);

        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();

        boolean brakeMode = true;
        boolean lastBumperState = false;
        boolean lastX = false;

        while (opModeIsActive()) {
            rotaryspin.setTargetPosition(x); //targetPos
            //rotarykick.setPosition(0);
            boolean bumper = gamepad1.left_bumper;
            if (bumper && !lastBumperState) brakeMode = !brakeMode;
            lastBumperState = bumper;

            if (gamepad1.x && !lastX && !xActionRunning) {
                xActionRunning = true;
                new Thread (() -> {
                    int startPos = rotaryspin.getCurrentPosition();
                    int targetPos = startPos + TARGET_TICKS;
                    x = x+475;
                    rotaryspin.setTargetPosition(x); //targetPos
                    rotaryspin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rotaryspin.setPower(1.0);

                    while (opModeIsActive() && rotaryspin.isBusy()) {
                        telemetry.addData("rotaryspin Target", targetPos);
                        telemetry.addData("rotaryspin Pos", rotaryspin.getCurrentPosition());
                        telemetry.update();
                    
                    }

                    rotaryspin.setPower(0);
                    rotaryspin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    xActionRunning = false;
                }).start();
            }
            lastX = gamepad1.x;

            if (gamepad1.y && !lastY) {
                launcherPower = launcherPower == 0 ? 1.0 : 0.0;
                launcher.setPower(launcherPower);
            }
            lastY = gamepad1.y;

            if (gamepad1.b && !lastB && !bActionRunning) {
                bActionRunning = true;
                new Thread(() -> {
                    pushup.setPosition(5.0 / 180.0);
                    sleep(600);
                    pushup.setPosition(160.0 / 180.0);
                    
                    
                    int startPos = rotaryspin.getCurrentPosition();
                    int targetPos = startPos + TARGET_TICKS;
                    x = x+475;
                    rotaryspin.setTargetPosition(x); //targetPos
                    rotaryspin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rotaryspin.setPower(1.0);

                    while (opModeIsActive() && rotaryspin.isBusy()) {
                        telemetry.addData("rotaryspin Target", targetPos);
                        telemetry.addData("rotaryspin Pos", rotaryspin.getCurrentPosition());
                        telemetry.update();
                    
                    }

                    rotaryspin.setPower(0);
                    rotaryspin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    bActionRunning = false;
                }).start();
                
                
            }
            lastB = gamepad1.b;

            //rotarykick.setPosition(0);

            if (gamepad1.a && !lastA && !aActionRunning) {
                aActionRunning = true;
                new Thread(() -> {
                    launcher.setPower(0.3);
                    sleep(1000);
                    rotarykick.setPosition(0.2);
                    sleep(500);
                    rotarykick.setPosition(0);
                    launcher.setPower(0.0);
                    
                    aActionRunning = false;
                }).start();
            }
                    
            lastA = gamepad1.a;
                
            if (gamepad1.left_trigger > 0.05) {
                motorFL.setPower(0);
                motorFR.setPower(0);
                motorBL.setPower(0);
                motorBR.setPower(0);
                intake.setPower(0);

                motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } else {
                DcMotor.ZeroPowerBehavior behavior =
                        brakeMode ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT;

                motorFL.setZeroPowerBehavior(behavior);
                motorFR.setZeroPowerBehavior(behavior);
                motorBL.setZeroPowerBehavior(behavior);
                motorBR.setZeroPowerBehavior(behavior);
                intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

                double forward = -gamepad1.left_stick_y;
                double strafe = gamepad1.left_stick_x;
                double turn = gamepad1.right_stick_x;

                double leftPower = forward + turn;
                double rightPower = forward - turn;
                double frontPower = strafe + turn;
                double backPower = -strafe + turn;

                motorFL.setPower(leftPower);
                motorBR.setPower(rightPower);
                motorFR.setPower(frontPower);
                motorBL.setPower(backPower);

                double intakePower = gamepad1.right_trigger;
                intake.setPower(intakePower * -1);
            }

            telemetry.addData("Brake Mode", brakeMode ? "BRAKE" : "FLOAT");
            telemetry.addData("rotaryspin Pos", rotaryspin.getCurrentPosition());
            telemetry.addData("Launcher Power", launcherPower);
            telemetry.update();

            sleep(30);
        }
    }
}
