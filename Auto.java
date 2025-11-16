package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "BackupAuto")
public class Auto extends LinearOpMode {

    // Drive motors
    private DcMotor motorFL, motorFR, motorBL, motorBR;
    
    // Launcher motor
    private DcMotor launcher;
    
    // Servo
    private Servo pushupservo;
    
    // Timing variables (in milliseconds)
    private final double BACKUP_TIME = 4700;        // Time to backup
    private final double SPINUP_WAIT = 2500;        // Wait for motor to spin up
    private final double SHOOT_WAIT = 1200;          // Wait between shots
    private final double LAUNCHER_POWER = 0.92;      // Launcher motor power (0.0 to 1.0)
    private final double BACKUP_POWER = 0.26;        // Drive power for backing up

    @Override
    public void runOpMode() {
        
        // Initialize hardware
        motorFL = hardwareMap.get(DcMotor.class, "motor3");
        motorFR = hardwareMap.get(DcMotor.class, "motor2");
        motorBL = hardwareMap.get(DcMotor.class, "motor4");
        motorBR = hardwareMap.get(DcMotor.class, "motor1");
        
        launcher = hardwareMap.get(DcMotor.class, "launcher");
        pushupservo = hardwareMap.get(Servo.class, "pushupservo");
        
        // Set motor directions
        motorFL.setDirection(DcMotor.Direction.FORWARD);
        motorBL.setDirection(DcMotor.Direction.FORWARD);
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.REVERSE);
        
        launcher.setDirection(DcMotor.Direction.FORWARD);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Set servo to rest position
        pushupservo.setPosition(0.4);
        
        telemetry.addLine("Ready to start!");
        telemetry.update();
        waitForStart();
        
        // === AUTONOMOUS SEQUENCE ===
        
        // 1. Backup
        telemetry.addLine("Backing up...");
        telemetry.update();
        setDrivePower(-BACKUP_POWER);  // Negative for backward
        sleep((long)BACKUP_TIME);
        setDrivePower(0);
        
        // 2. Spin up launcher
        telemetry.addLine("Spinning up launcher...");
        telemetry.update();
        launcher.setPower(LAUNCHER_POWER);
        sleep((long)SPINUP_WAIT);
        
        // 3. Shoot ball 1
        telemetry.addLine("Shooting ball 1...");
        telemetry.update();
        shootBall();
        sleep((long)SHOOT_WAIT);
        
        // 4. Shoot ball 2
        telemetry.addLine("Shooting ball 2...");
        telemetry.update();
        shootBall();
        sleep((long)SHOOT_WAIT);
        
        // 5. Shoot ball 3
        telemetry.addLine("Shooting ball 3...");
        telemetry.update();
        shootBall();
        sleep((long)SHOOT_WAIT);
        
        // 6. Stop launcher
        telemetry.addLine("Stopping launcher...");
        telemetry.update();
        launcher.setPower(0);
        
        telemetry.addLine("Autonomous complete!");
        telemetry.update();
    }
    
    // Helper method to set all drive motors to same power
    private void setDrivePower(double power) {
        motorFL.setPower(power);
        motorFR.setPower(power);
        motorBL.setPower(power);
        motorBR.setPower(power);
    }
    
    // Helper method to actuate servo (shoot one ball)
    private void shootBall() {
        pushupservo.setPosition(0);
        sleep(500);
        pushupservo.setPosition(0.4);
    }
}
