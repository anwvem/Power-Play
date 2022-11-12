package org.firstinspires.ftc.teamcode.glowCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class testing extends LinearOpMode {
    private final HardwareMapping robot = new HardwareMapping();


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        if (opModeIsActive()) {

            /*robot.rightFront.setPower(0.3);
            sleep(5000);

            robot.rightRear.setPower(0.3);
            sleep(5000);

            robot.leftFront.setPower(0.3);
            sleep(5000);

            robot.leftRear.setPower(0.3);
            sleep(5000);

             */

            robot.driveAtDirection(0, 1000, 0.3);



        }
    }
}

