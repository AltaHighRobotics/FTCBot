package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import android.util.Size;
import java.util.List;

@Autonomous(name = "TagSeek")
public class TagSeek extends AbstractMoveAPI {
    private VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;

    @Override
    public void runOpMode() {
        initHardware();

        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "c720-1"))
                .setCameraResolution(new Size(320, 240))
                .enableLiveView(false)
                .addProcessor(tagProcessor)
                .build();

        waitForStart();

        while (opModeIsActive()) {
            List<AprilTagDetection> detections = tagProcessor.getDetections();
            if (detections.isEmpty()) {
                stopAll();
                telemetry.addData("Tag", "none");
                telemetry.update();
                continue;
            }

            AprilTagDetection d = detections.get(0);
            double yErr = d.ftcPose.y - 10.0;
            double xErr = d.ftcPose.x;
            double yawErr = d.ftcPose.yaw;

            double movePower = Math.min(0.5, Math.abs(yErr) / 20.0 + 0.15);
            double strafePower = Math.min(0.4, Math.abs(xErr) / 15.0 + 0.1);
            double turnPower = Math.min(0.3, Math.abs(yawErr) / 30.0 + 0.1);

            if (Math.abs(yErr) > 1.0) {
                if (yErr > 0) moveBackward(movePower, 50);
                else moveForward(movePower, 50);
            }

            if (Math.abs(xErr) > 1.0) {
                if (xErr > 0) moveRight(strafePower, 50);
                else moveLeft(strafePower, 50);
            }

            if (Math.abs(yawErr) > 3.0) {
                if (yawErr > 0) rotate(-turnPower, 30);
                else rotate(turnPower, 30);
            }

            telemetry.addData("X", "%.1f", d.ftcPose.x);
            telemetry.addData("Y", "%.1f", d.ftcPose.y);
            telemetry.addData("Yaw", "%.1f", d.ftcPose.yaw);
            telemetry.update();
        }
    }
}
