/*
Copyright 2025 Alta High School Robotics Team

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
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "Manual Driving")

public class MainBase extends LinearOpMode {
    private Blinker control_Hub;
    private DcMotor ml;
    private DcMotor mr;
    private boolean reverse;
    @Override
    public void runOpMode() {
        // initialization code
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        ml = hardwareMap.get(DcMotor.class, "motor1");
        mr = hardwareMap.get(DcMotor.class, "motor2");
        telemetry.addData("Reverse", "OFF");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // waits till you press the start button
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // about 60hz polling
            sleep(16);
            if (reverse) {
                // dw the control station does some kind of deadzoning so we're fine
                if (gamepad1.left_stick_x != 0) {
                    ml.setPower(gamepad1.left_stick_x * -1);
                    mr.setPower(gamepad1.left_stick_x * -1);
                    } else {
                       ml.setPower(gamepad1.left_stick_y * -1);
                       mr.setPower(gamepad1.left_stick_y);
                    }
            } else {
                if (gamepad1.left_stick_x != 0) {
                        ml.setPower(gamepad1.left_stick_x);
                        mr.setPower(gamepad1.left_stick_x);
                    } else {
                        ml.setPower(gamepad1.left_stick_y);
                        mr.setPower(gamepad1.left_stick_y * -1);
                    }
                
            }
            if (gamepad1.left_bumper) {
                reverse = true;
                telemetry.addData("Reverse", "ON");
            } else {
                reverse = false;
                telemetry.addData("Reverse", "OFF");
            }
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }
}
