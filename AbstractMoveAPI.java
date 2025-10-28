package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public abstract class AbstractMoveAPI extends LinearOpMode {
    protected DcMotor mv1, mv2, mh1, mh2;

    protected void initHardware() {
        mv1 = hardwareMap.get(DcMotor.class, "motor1");
        mv2 = hardwareMap.get(DcMotor.class, "motor2");
        mh1 = hardwareMap.get(DcMotor.class, "motor3");
        mh2 = hardwareMap.get(DcMotor.class, "motor4");
    }

    protected void stopAll() {
        mv1.setPower(0);
        mv2.setPower(0);
        mh1.setPower(0);
        mh2.setPower(0);
    }

    protected void moveForward(double power, long timeMs) {
        mv1.setPower(power);
        mv2.setPower(-power);
        mh1.setPower(0);
        mh2.setPower(0);
        sleep(timeMs);
        stopAll();
    }

    protected void moveBackward(double power, long timeMs) {
        mv1.setPower(-power);
        mv2.setPower(power);
        mh1.setPower(0);
        mh2.setPower(0);
        sleep(timeMs);
        stopAll();
    }

    protected void moveRight(double power, long timeMs) {
        mh1.setPower(power);
        mh2.setPower(-power);
        mv1.setPower(0);
        mv2.setPower(0);
        sleep(timeMs);
        stopAll();
    }

    protected void moveLeft(double power, long timeMs) {
        mh1.setPower(-power);
        mh2.setPower(power);
        mv1.setPower(0);
        mv2.setPower(0);
        sleep(timeMs);
        stopAll();
    }

    protected void rotate(double power, long timeMs) {
        mv1.setPower(-power);
        mv2.setPower(-power);
        mh1.setPower(-power);
        mh2.setPower(-power);
        sleep(timeMs);
        stopAll();
    }
}
