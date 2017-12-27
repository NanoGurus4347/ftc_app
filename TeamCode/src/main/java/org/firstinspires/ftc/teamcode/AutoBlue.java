package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Rithv on 12/26/2017.
 */

@Autonomous
public class AutoBlue extends Auto2017Methods {

    @Override
    public void runOpMode() throws InterruptedException {

        //motors
        initRobot();

//score jewel and drive into safe zone -- 45 pts
        waitForStart();

        scoreJewelBlue();

        stop();

    }
    
}
