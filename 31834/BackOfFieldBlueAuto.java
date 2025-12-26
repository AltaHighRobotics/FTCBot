/*
Copyright 2025 FIRST Tech Challenge Team 31834

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.dfrobot.HuskyLens;

@Autonomous
public class BackOfFieldBlueAuto extends LinearOpMode {
    private static final double TICKS_PER_REV = 1425.1;
    private DcMotor motorFL, motorFR, motorBL, motorBR;
    private DcMotor intake, rotaryspin;
    private DcMotorEx launcher;  
    private PIDController launcherPID;

    private double targetLauncherVelocity = 3100; 
    private double currentVelocity = 0;
    private double pidPower = 0;

    private Servo pushup, rotarykick;
    int x = -15; 
    double turn = 0;
    int RotateStage = 1;
    private HuskyLens husky;

    // HuskyLens detection variables
    private volatile boolean tagVisible = false;
    private volatile double tagX = 0;
    int ObId = 0;

    @Override
    public void runOpMode() {

        motorFL = hardwareMap.get(DcMotor.class, "motor2");
        motorBR = hardwareMap.get(DcMotor.class, "motor1");
        motorFR = hardwareMap.get(DcMotor.class, "motor3");
        motorBL = hardwareMap.get(DcMotor.class, "motor4");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        rotarykick = hardwareMap.get(Servo.class, "rotarykick");
        pushup = hardwareMap.get(Servo.class, "pushup");
        rotaryspin = hardwareMap.get(DcMotor.class, "rotaryspin");
        husky = hardwareMap.get(HuskyLens.class, "husky");

        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.FORWARD);
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        launcher.setDirection(DcMotor.Direction.FORWARD);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        boolean brakeMode = true;
        rotarykick.setPosition(0);
        // ================== HUSKYLENS DETECTION THREAD WITH TELEMETRY ==================
        new Thread(() -> {
            while (opModeIsActive()) {
                
                HuskyLens.Block[] blocks = husky.blocks();
                sleep(20);
                if (blocks == null) {
                    telemetry.addLine("blocks() returned null");
                    telemetry.update();
                    sleep(50);
                    continue;
                }
                if (blocks.length == 0) {
                    telemetry.addLine("blocks array is empty");
                    telemetry.update();
                    sleep(50);
                    continue;
                }

                HuskyLens.Block found = null;
                for (HuskyLens.Block b : blocks) {
                    if (b == null) continue;
                    telemetry.addData("Detected block", "ID=" + b.id + ", X=" + b.x + ", Y=" + b.y +
                                                      ", Width=" + b.width + ", Height=" + b.height);
                          
                    if (b.id == 1) { // change ID here if needed
                        found = b;
                        
                        break;
                    } else {
                        if (b.id > 2) {
                            ObId = b.id;
                            telemetry.addData("ob tag is" , ObId);
                            telemetry.update();
                        }
                    }
                }
                
                if (found != null) {
                    tagVisible = true;
                    tagX = found.x;
                    telemetry.addData("Found tag", "ID=" + found.id + ", X=" + found.x);
                } else {
                    tagVisible = false;
                    telemetry.addLine("No tag with ID 2 found");
                }
                telemetry.update();
                sleep(50);
                telemetry.addData("ob tag is" , ObId);
                telemetry.update();
            }
        }).start();

        // ================== MAIN AUTO LOOP ==================
        while (opModeIsActive()) {

            // Move rotary spin
            rotaryspin.setTargetPosition(x);
            rotaryspin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rotaryspin.setPower(1.0);
            while (opModeIsActive() && rotaryspin.isBusy()) {
                telemetry.addData("rotaryspin Pos", rotaryspin.getCurrentPosition());
                telemetry.update();
            }
            rotaryspin.setPower(0);
            rotaryspin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Initial move
            motorFL.setPower(0.8);
            motorFR.setPower(0);
            motorBL.setPower(0);
            motorBR.setPower(0.8);
            sleep(1360);
            motorFL.setPower(0);
            motorFR.setPower(0);
            motorBL.setPower(0);
            motorBR.setPower(0);
            sleep(400);
            // Launcher PID setup and small backward nudge
            launcherPID = new PIDController(0.0250, 0.0007, 0.0003);
            motorFL.setPower(-0.7);
            motorFR.setPower(-0.7);
            motorBL.setPower(-0.7);
            motorBR.setPower(-0.7);
            sleep(200);

            motorFL.setPower(0);
            motorFR.setPower(0);
            motorBL.setPower(0);
            motorBR.setPower(0);
            sleep(200);

            // ================== WAIT FOR TAG ==================
            while (opModeIsActive() && !tagVisible) {
                telemetry.addLine("Waiting for AprilTag ID 2...");
                telemetry.update();
                sleep(50);
            }

            // ================== ALIGNMENT ==================
            if (tagVisible) {
                double cx = 180.0;
                double acceptableError = 15.0;
                double errorX = tagX - cx;

                while (opModeIsActive() &&
                        (errorX < -acceptableError || errorX > acceptableError)) {

                    turn = -(errorX / cx) * 0.9;

                    motorFL.setPower(-turn);
                    motorFR.setPower(-turn);
                    motorBL.setPower(-turn);
                    motorBR.setPower(turn);

                    telemetry.addData("Turning to align", "errorX=" + errorX + ", turn=" + turn);
                    telemetry.update();

                    // Update errorX from HuskyLens
                    HuskyLens.Block[] updated = husky.blocks();
                    if (updated != null && updated.length > 0 && updated[0] != null) {
                        errorX = updated[0].x - cx;
                    } else {
                        break; // stop if no blocks detected
                    }
                }

                motorFL.setPower(0);
                motorFR.setPower(0);
                motorBL.setPower(0);
                motorBR.setPower(0);
            }

            // ================== REST OF AUTO (unchanged) ==================
            DcMotor.ZeroPowerBehavior behavior =
                brakeMode ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT;

            motorFL.setZeroPowerBehavior(behavior);
            motorFR.setZeroPowerBehavior(behavior);
            motorBL.setZeroPowerBehavior(behavior);
            motorBR.setZeroPowerBehavior(behavior);

            for (int f = 0; f < 4; f++) {
                
              
                if (ObId == 5) {
                    if (RotateStage == 1) {
                        x = 460; //was -15
                    } else {
                        if (RotateStage == 2) {
                            x = -15; //was 460
                        } else {
                            x = 935; 
                        }  
                    }
                } else {
                    if (ObId == 3) {
                        if (RotateStage == 1) {
                        x = -15;
                    } else {
                        if (RotateStage == 2) {
                            x = 460;
                        } else {
                            x = 935;
                        }  
                    }
                    } else {
                        if (RotateStage == 1) {
                        x = -15;
                    } else {
                        if (RotateStage == 2) {
                            x = 935;
                        } else {
                            x = 460;
                        }  
                    }
                    }
                }
                RotateStage += 1;
                //x += 475;
                rotaryspin.setTargetPosition(x);
                rotaryspin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rotaryspin.setPower(1.0);
                while (opModeIsActive() && rotaryspin.isBusy()) {
                    telemetry.addData("rotaryspin Pos", rotaryspin.getCurrentPosition());
                    telemetry.update();
                }
                rotaryspin.setPower(0);
                rotaryspin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                
                  new Thread(() -> {
                    sleep(1000);
                    rotarykick.setPosition(0.2);
                    sleep(500);
                    rotarykick.setPosition(0);
                }).start();

                for (int i = 0; i < 100; i++) {
                    currentVelocity = launcher.getVelocity();
                    pidPower = launcherPID.update(targetLauncherVelocity, currentVelocity);
                    pidPower = Math.max(Math.min(pidPower, 0.3), 0.0);
                    launcher.setPower(pidPower);
                    sleep(20);
                }

                new Thread(() -> {
                    sleep(2000);
                    pushup.setPosition(5.0 / 180.0);
                    sleep(600);
                    pushup.setPosition(160.0 / 180.0);
                }).start();

                for (int i = 0; i < 100; i++) {
                    currentVelocity = launcher.getVelocity();
                    pidPower = launcherPID.update(targetLauncherVelocity, currentVelocity);
                    pidPower = Math.max(Math.min(pidPower, 0.9), 0.0);
                    launcher.setPower(pidPower);
                    sleep(20);
                }
                
                pidPower = 0.0;
                launcher.setPower(pidPower);
                rotarykick.setPosition(0);
                sleep(100);
            }

            sleep(10000);
        }
    }
}
