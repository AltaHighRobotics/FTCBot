package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class LauncherSubsystem {

    // --- Hardware ---
    private DcMotorEx launcher;   
    private Servo pushupservo;

    public static double P = 10.0;
    public static double I = 3.0;
    public static double D = 0.0;
    public static double F = 14.0; // Feedforward is critical for velocity

    // --- Motor Constants ---
    private static final double LOW_RPM = 2800;
    private static final double HIGH_RPM = 5600;
    private static final double TPR = 28; // Ticks Per Revolution

    // --- Servo State Machine (Unchanged) ---
    private enum ServoState { IDLE, POSITION1, DELAY1, POSITION2, DELAY2, POSITION3 }
    private ServoState servoState = ServoState.IDLE;
    private ElapsedTime servoTimer = new ElapsedTime();
    private long lastAPressTime = 0;
    private static final double SERVO_POS_1 = 0.3;
    private static final double SERVO_POS_2 = 0.0;
    private static final double SERVO_POS_3_IDLE = 0.4;
    private static final long SERVO_DELAY_MS = 500;
    private static final long SERVO_COOLDOWN_MS = 2000;

    // --- State Variables ---
    private double currentTargetRPM = 0;

    /**
     * Initializes the launcher and servo hardware.
     * @param hardwareMap The OpMode's hardware map.
     */
    public void init(HardwareMap hardwareMap) {
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        pushupservo = hardwareMap.get(Servo.class, "pushupservo");

        // Set motor mode to use on-board PID
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set our PIDF coefficients
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(P, I, D, F));

        // Initialize Servo
        servoTimer.reset();
        servoState = ServoState.IDLE;
        pushupservo.setPosition(SERVO_POS_3_IDLE); 
    }


    /**
     * Overrides the launcher's target RPM with a specific value.
     */
    public void setTargetRPM(double rpm) {
        this.currentTargetRPM = rpm;
        // Call the private method to update motor velocity
        setMotorVelocity(currentTargetRPM);
    }

    /**
     * Helper method to convert RPM to Ticks per Second and set motor velocity.
     */
    private void setMotorVelocity(double rpm) {
        double ticksPerSecond = (rpm * TPR) / 60.0;
        
        // 8. This one command tells the motor to hold the speed
        launcher.setVelocity(ticksPerSecond);
    }

    /**
     * Attempts to start the servo "push" sequence. 
     */
    public void triggerServoSequence() {
        long now = System.currentTimeMillis();
        if (servoState == ServoState.IDLE && (now - lastAPressTime >= SERVO_COOLDOWN_MS)) {
            lastAPressTime = now;
            servoState = ServoState.POSITION1;
            servoTimer.reset();
        }
    }

    /**
     * This method must be called every loop in the OpMode.
     */
    public void update() {
        
        switch (servoState) {
            case POSITION1:
                pushupservo.setPosition(SERVO_POS_1);
                servoState = ServoState.DELAY1;
                servoTimer.reset();
                break;
            
            case DELAY1:
                if (servoTimer.milliseconds() >= SERVO_DELAY_MS) {
                    servoState = ServoState.POSITION2;
                    servoTimer.reset();
                }
                break;
            
            case POSITION2:
                pushupservo.setPosition(SERVO_POS_2);
                servoState = ServoState.DELAY2;
                servoTimer.reset();
                break;
            
            case DELAY2:
                if (servoTimer.milliseconds() >= SERVO_DELAY_MS) {
                    servoState = ServoState.POSITION3;
                    servoTimer.reset();
                }
                break;
            
            case POSITION3:
                pushupservo.setPosition(SERVO_POS_3_IDLE);
                servoState = ServoState.IDLE;
                break;
            
            case IDLE:
                break;
        }
    }

    // --- Telemetry Methods ---

    public double getTargetRPM() {
        return currentTargetRPM;
    }

    // Optional: Get the *actual* current RPM from the motor
    public double getCurrentRPM() {
        double ticksPerSecond = launcher.getVelocity();
        return (ticksPerSecond * 60.0) / TPR;
    }
}
