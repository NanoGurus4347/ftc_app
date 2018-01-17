package org.firstinspires.ftc.teamcode;

/**
 * Created by Rithv on 12/26/2017.
 */

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class GyroTurn extends LinearOpMode{
    DcMotor leftFront = null;  //HAS ENCODER
    DcMotor rightFront = null; //HAS ENCODER
    DcMotor leftBack = null;
    DcMotor rightBack = null;

    ModernRoboticsI2cGyro modernRoboticsI2cGyro;

    // can change the value of target, I just put 90 as an example
    int target = 90;
    // used for calibration
    ElapsedTime timer = new ElapsedTime();

    public void runOpMode() throws InterruptedException {

        leftFront  = hardwareMap.get(DcMotor.class, "ld1");
        rightFront = hardwareMap.get(DcMotor.class, "rd1");
        leftBack  = hardwareMap.get(DcMotor.class, "ld2");
        rightBack = hardwareMap.get(DcMotor.class, "rd2");

        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");



        // Start calibrating the gyro.
        telemetry.log().add("Gyro Calibrating. Do Not Move!");
        modernRoboticsI2cGyro.calibrate();

        // Wait until the gyro calibration is complete
        timer.reset();
        while (!isStopRequested() && modernRoboticsI2cGyro.isCalibrating())  {
            telemetry.addData("calibrating", "%s", Math.round(timer.seconds())%2==0 ? "|.." : "..|");
            telemetry.update();
            sleep(50);
        }

        telemetry.log().clear(); telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear(); telemetry.update();

        int currentPosition = modernRoboticsI2cGyro.getIntegratedZValue();
        int initialPosition = currentPosition;

            waitForStart();

        telemetry.addData("Initial Position", currentPosition);
        telemetry.update();

            while (opModeIsActive()) {
                //currentPosition needs to be updated every time in the loop
                currentPosition = modernRoboticsI2cGyro.getIntegratedZValue();
                telemetry.addData("Current Position", currentPosition);
                telemetry.update();
                // If the initial position(not updating) - the currentPosition(always updating) < 90, move motors
                if (currentPosition - initialPosition < target) {
                    //telemetry.addData("Almost there!", currentPosition );
                    telemetry.update();
                    setPowerSides(0.2, -0.2);
                }
                // If the condition is true, then stop the motors
                else {
                    telemetry.addData("Reached!", currentPosition );
                    telemetry.update();
                    leftFront.setPower(0);
                    leftBack.setPower(0);
                    rightFront.setPower(0);
                    rightBack.setPower(0);
                    break;
                }
            }
        }
    }


