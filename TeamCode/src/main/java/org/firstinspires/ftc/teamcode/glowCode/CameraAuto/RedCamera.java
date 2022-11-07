package org.firstinspires.ftc.teamcode.glowCode.CameraAuto;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.glowCode.HardwareMapping;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous (name = "Red Camera Auto")
public class RedCamera extends LinearOpMode {
    private final HardwareMapping robot = new HardwareMapping();
    EOAprilTags.pos pos;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        if (opModeIsActive()) {
            telemetry.addData("position",pos);
            telemetry.update();
            ElapsedTime runtime2 = new ElapsedTime();
            switch (pos) {

                case LEFT:
                    //move to carousel
                    robot.driveAtDirection(0, 100, .3);
                    robot.driveAtDirection(270, 2000, .3);
                    break;

                case MIDDLE:
                    robot.driveAtDirection(0, 2000, .3);
                    break;

                case RIGHT:
                    //move right about a foot
                    robot.driveAtDirection(90, 500, .3);
                    robot.driveAtDirection(0, 2800, .3);

                    break;




            }

        }
    }
}