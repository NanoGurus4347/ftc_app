package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.I2cAddr;

/**
 * Created by Rithv on 12/24/2017.
 **/
//Github
@Autonomous
public class GyroAlignment extends LinearOpMode {
    ModernRoboticsI2cGyro modernRoboticsI2cGyro;

    int target = 90;

    public void runOpMode() throws InterruptedException {

        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        modernRoboticsI2cGyro.setI2cAddress(I2cAddr.create8bit(0x20));

        int currentPosition = modernRoboticsI2cGyro.getIntegratedZValue();
        int initialPosition = currentPosition;

        telemetry.addData("Current Position", currentPosition);

        waitForStart();
        while (opModeIsActive()) {
            //currentPosition needs to be updated every time in the loop
            currentPosition = modernRoboticsI2cGyro.getIntegratedZValue();
            telemetry.addData("Current Position", currentPosition);
            telemetry.update();
            // If the initial position(not updating) - the currentPosition(always updating) < 90, move motors
            if (currentPosition - initialPosition < target) {
                telemetry.addData("Almost there!", currentPosition );
            }
            // If the condition is true, then stop the motors
            else {
                telemetry.addData("Reached!", currentPosition );

            }
        }
    }
}



