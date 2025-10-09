package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Demo")
public class AutoDemo extends AbstractMoveAPI {
    @Override
    public void runOpMode() {
        initHardware();
        waitForStart();
        moveForward(0.5, 1000);
        moveRight(0.5, 800);
        rotate(0.5, 600);
        moveBackward(0.5, 1000);
    }
}
