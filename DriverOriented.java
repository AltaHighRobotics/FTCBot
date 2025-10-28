package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "DriverOrientedDriving")
public class DriverOriented extends LinearOpMode {
    private DcMotor motorFL, motorFR, motorBL, motorBR;
    private IMU imu;
    private double initialHeading = 0;

    @Override
    public void runOpMode() {
        motorFL = hardwareMap.get(DcMotor.class, "motor2");
        motorBR = hardwareMap.get(DcMotor.class, "motor1");
        motorFR = hardwareMap.get(DcMotor.class, "motor3");
        motorBL = hardwareMap.get(DcMotor.class, "motor4");

        motorFL.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.FORWARD);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
        IMU.Parameters parameters = new IMU.Parameters(orientation);
        imu.initialize(parameters);

        telemetry.addLine("IMU Initialized");
        telemetry.update();

        waitForStart();

        initialHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        boolean brakeMode = true;
        boolean lastBumperState = false;

        DcMotor.ZeroPowerBehavior behavior = DcMotor.ZeroPowerBehavior.BRAKE;
        motorFL.setZeroPowerBehavior(behavior);
        motorFR.setZeroPowerBehavior(behavior);
        motorBL.setZeroPowerBehavior(behavior);
        motorBR.setZeroPowerBehavior(behavior);

        while (opModeIsActive()) {
            boolean bumper = gamepad1.left_bumper;
            if (bumper && !lastBumperState) {
                brakeMode = !brakeMode;
                behavior = brakeMode ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT;
                motorFL.setZeroPowerBehavior(behavior);
                motorFR.setZeroPowerBehavior(behavior);
                motorBL.setZeroPowerBehavior(behavior);
                motorBR.setZeroPowerBehavior(behavior);
            }
            lastBumperState = bumper;

            double y = gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

            double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double heading = currentHeading - initialHeading;
            heading = (heading + Math.PI) % (2 * Math.PI) - Math.PI;

            double rotatedX = x * Math.cos(heading) - y * Math.sin(heading);
            double rotatedY = x * Math.sin(heading) + y * Math.cos(heading);

            double denominator = Math.max(Math.abs(rotatedY) + Math.abs(rotatedX) + Math.abs(rx), 1.0);
            double powerFL = (rotatedY + rx) / denominator;
            double powerBR = (rotatedY - rx) / denominator;
            double powerFR = (rotatedX - rx) / denominator;
            double powerBL = (rotatedX + rx) / denominator;

            motorFL.setPower(powerFL);
            motorBR.setPower(powerBR);
            motorFR.setPower(powerFR);
            motorBL.setPower(powerBL);

            telemetry.addData("Brake Mode", brakeMode ? "BRAKE" : "FLOAT");
            telemetry.addData("Current Heading (deg)", Math.toDegrees(currentHeading));
            telemetry.addData("Driver Forward (deg)", Math.toDegrees(initialHeading));
            telemetry.addData("Relative Heading (deg)", Math.toDegrees(heading));
            telemetry.addData("Joystick X", x);
            telemetry.addData("Joystick Y", y);
            telemetry.addData("FL", powerFL);
            telemetry.addData("FR", powerFR);
            telemetry.addData("BL", powerBL);
            telemetry.addData("BR", powerBR);
            telemetry.update();
            sleep(20);
        }
    }
}
