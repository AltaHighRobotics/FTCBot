package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.openftc.easyopencv.*; 
import java.util.List; 

@TeleOp(name = "Shayden Bot")
public class ShaydenBot extends OpMode {
    private DriveSubsystem drivetrain;
    private LauncherSubsystem launcher;

    private boolean lastA = false;
    private boolean lastStart = false;
    private boolean lastDpadLeft = false;
    private boolean lastLeftBumper = false;
    
    @Override
    public void init() {
        drivetrain = new DriveSubsystem();
        drivetrain.init(hardwareMap);
        
        launcher = new LauncherSubsystem();
        launcher.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        drivetrain.resetHeading();
    }

    @Override
    public void loop() {
        
        // --- Gamepad 1 Inputs ---
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        // --- Drivetrain Controls ---
        drivetrain.driveFeildOriented(y, x, turn); 

        // Reset IMU Heading
        boolean startPressed = gamepad1.start;
        if (startPressed && !lastStart) {
            drivetrain.resetHeading();
        }
        lastStart = startPressed;

        boolean dpadLeftPressed = gamepad1.dpad_left;
        if (dpadLeftPressed && !lastDpadLeft) {
            drivetrain.resetHeading();
        }
        lastDpadLeft = dpadLeftPressed;
        
        boolean leftBumperPressed = gamepad1.left_bumper;
        if (leftBumperPressed && !lastLeftBumper) {
            drivetrain.toggleBrakeMode();
        }
        lastLeftBumper = leftBumperPressed;
        
        
        launcher.setTargetRPM(gamepad1.right_trigger * 6000);
        
        if (gamepad1.a && !lastA) {
            launcher.triggerServoSequence();
        }
        lastA = gamepad1.a;
        
        launcher.update();

        telemetry.addData("motor RPM", launcher.getCurrentRPM());
        telemetry.addData("Heading(deg)", drivetrain.getBotHeadingDegrees());
        telemetry.addData("Motor mode", drivetrain.getMotorModeString());
        telemetry.update();
    }

    @Override
    public void stop() {
        drivetrain.driveFeildOriented(0, 0, 0); // Stop drive motors
        launcher.setTargetRPM(0); // Stop launcher
        launcher.update();
    }
}
