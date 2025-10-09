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
    private DcMotor mh1;
    private DcMotor mh2;
    private DcMotor mv1;
    private DcMotor mv2;
    private boolean reverse;
    //private boolean reverse_Rightstick;
    @Override
    public void runOpMode() {
        // initialization code
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        mv1 = hardwareMap.get(DcMotor.class, "motor1");
        mv2 = hardwareMap.get(DcMotor.class, "motor2");
        mh1 = hardwareMap.get(DcMotor.class, "motor3");
        mh2 = hardwareMap.get(DcMotor.class, "motor4");
        telemetry.addData("Reverse", "OFF");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // waits till you press the start button
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // about 60hz polling
            // technically this should be 16.66667 (67!)or whatever but I doubt half of a millisecond makes much of a difference
            sleep(16);
            if (reverse) {
                // dw the control station does some kind of deadzoning so we're fine
                mv1.setPower(gamepad1.left_stick_y * -1);
                mv2.setPower(gamepad1.left_stick_y);
                mh1.setPower(gamepad1.left_stick_x * -1);
                mh2.setPower(gamepad1.left_stick_x);

            } else {
                mv1.setPower(gamepad1.left_stick_y);
                mv2.setPower(gamepad1.left_stick_y * -1);
                mh1.setPower(gamepad1.left_stick_x);
                mh2.setPower(gamepad1.left_stick_x * -1);
            }
            if  (gamepad1.right_stick_x > 0){
                mv1.setPower(gamepad1.right_stick_x*-1 );
                mv2.setPower(gamepad1.right_stick_x*-1);
                mh1.setPower(gamepad1.right_stick_x *-1);
                mh2.setPower(gamepad1.right_stick_x *-1);
            } 
            if (gamepad1.right_stick_x < 0) {
                mv1.setPower(gamepad1.right_stick_x * -1);
                mv2.setPower(gamepad1.right_stick_x * -1 );
                mh1.setPower(gamepad1.right_stick_x * -1);
                mh2.setPower(gamepad1.right_stick_x * -1);
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
