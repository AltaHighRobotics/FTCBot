package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DriveSubsystem {

    private DcMotor motorFL, motorFR, motorBL, motorBR;
    private IMU imu;
    private double initialHeading = 0.0;
    private boolean brakeMode = true; // Default to BRAKE mode

    public void init(HardwareMap hardwareMap) {
        motorFL = hardwareMap.get(DcMotor.class, "motor3");
        motorFR = hardwareMap.get(DcMotor.class, "motor2");
        motorBL = hardwareMap.get(DcMotor.class, "motor4");
        motorBR = hardwareMap.get(DcMotor.class, "motor1");

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        ));

        motorFL.setDirection(DcMotor.Direction.FORWARD);
        motorBL.setDirection(DcMotor.Direction.FORWARD);
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.REVERSE);

        setDriveMotorBehavior(brakeMode);
        resetHeading();
    }

    public void resetHeading() {
        initialHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public void toggleBrakeMode() {
        brakeMode = !brakeMode;
        setDriveMotorBehavior(brakeMode);
    }

    public void setDriveMotorBehavior(boolean isBrake) {
        DcMotor.ZeroPowerBehavior behavior = isBrake ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT;
        setDriveMotorBehavior(behavior);
    }

    public void setDriveMotorBehavior(DcMotor.ZeroPowerBehavior behavior) {
        motorFL.setZeroPowerBehavior(behavior);
        motorFR.setZeroPowerBehavior(behavior);
        motorBL.setZeroPowerBehavior(behavior);
        motorBR.setZeroPowerBehavior(behavior);
    }

    public void driveFeildOriented(double y, double x, double turn) {
        double rawYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double botHeading = normalizeRadians(rawYaw - initialHeading);

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
    }

    public void driveRobotOriented(double y, double x, double turn) {
        double fl = y + x + turn;
        double fr = y - x - turn;
        double bl = y - x + turn;
        double br = y + x - turn;

        double max = Math.max(1.0, Math.max(Math.abs(fl),
                Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
        
        motorFL.setPower(fl / max);
        motorFR.setPower(fr / max);
        motorBL.setPower(bl / max);
        motorBR.setPower(br / max);
    }


    public double getBotHeadingDegrees() {
        double rawYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        return Math.toDegrees(normalizeRadians(rawYaw - initialHeading));
    }

    public String getMotorModeString() {
        return brakeMode ? "BRAKE" : "FLOAT";
    }

    public boolean isBrakeMode() {
        return brakeMode;
    }
    private static double normalizeRadians(double angle) {
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle <= -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }
}
