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
        // Motor mappings
        motorFL = hardwareMap.get(DcMotor.class, "motor2");  // Front Left (vertical)
        motorBR = hardwareMap.get(DcMotor.class, "motor1");  // Back Right (vertical)
        motorFR = hardwareMap.get(DcMotor.class, "motor3");  // Front Right (horizontal)
        motorBL = hardwareMap.get(DcMotor.class, "motor4");  // Back Left (horizontal)

        // Motor directions
        motorFL.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.FORWARD);

        // Setup IMU
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

        // Set the driver's forward direction
        initialHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        while (opModeIsActive()) {
            // Read joystick input
            double y = gamepad1.left_stick_y;  // Forward/back
            double x = gamepad1.left_stick_x;   // Left/right strafe
            double rx = -gamepad1.right_stick_x; // Rotation

            // Get current heading
            double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Compute offset from initial driver-facing heading
            double heading = currentHeading - initialHeading;

            // Normalize to [-π, π]
            heading = (heading + Math.PI) % (2 * Math.PI) - Math.PI;

            // CORRECT ROTATION OF JOYSTICK INPUT (DO NOT NEGATE ANGLE)
            double rotatedX = x * Math.cos(heading) - y * Math.sin(heading);
            double rotatedY = x * Math.sin(heading) + y * Math.cos(heading);

            // Normalize motor power denominator
            double denominator = Math.max(Math.abs(rotatedY) + Math.abs(rotatedX) + Math.abs(rx), 1.0);

            // Motor power calculation for omni wheel configuration
            double powerFL = (rotatedY + rx) / denominator;
            double powerBR = (rotatedY - rx) / denominator;
            double powerFR = (rotatedX - rx) / denominator;
            double powerBL = (rotatedX + rx) / denominator;

            // Set powers to motors
            motorFL.setPower(powerFL);
            motorBR.setPower(powerBR);
            motorFR.setPower(powerFR);
            motorBL.setPower(powerBL);

            // Telemetry
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
