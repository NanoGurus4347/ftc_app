package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by vaish on 11/14/2017.
 */
@Autonomous(name="Blue Auto Middle")
public class AutoBlue extends Auto2017Methods{

    @Override
    public void runOpMode() throws InterruptedException {

        initRobot();

        waitForStart();
        int vuMark = vuScoreia(4);
        telemetry.addData("vuScoreia", vuMark);
        telemetry.update();
        sleep(1000);
        scoreJewelBlue();
        sleep(1000);

        setPower(.2);
        while(line.blue() < 10 && opModeIsActive()) {
            telemetry.addData("line", "not detected");
            telemetry.update();
        }
        while(line.blue() > 5 && opModeIsActive()) {
            telemetry.addData("line", "detected");
            telemetry.update();
        }
        halt();

        telemetry.addData("moveBack: ", vuMark*6);
        sleep(2000);
        //moveBackwardEncoders(6, .3);


        turn90Encoders();

        /*setPowerSides(-.4, -.2);
        sleep(1000);
        halt();*/

        collection.setPower(1);
        sleep(500);
        collection.setPower(0);

        moveBackwardEncoders(5, .3);

        setPower(-.3);
        sleep(1500);
        halt();

        moveBackwardEncoders(5, .3);

        /*score*/
        /*moveBack*/

        stop();

    }
}