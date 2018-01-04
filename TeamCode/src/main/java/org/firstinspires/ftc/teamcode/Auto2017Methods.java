package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


import android.text.InputFilter;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Locale;

/**
 * Created by Rithv on 12/26/2017.
 */

//Vuforia License
//AT1IefX/////AAAAGe7qBWfq2E2BmnY2RCELAlIGMKLOUEEXYPeKxkKghQTsCPGGs0xtZTnLSPrE6ExcuJ6WcUeXawPMLqGyDWP9a1YhinvHIaS+8cXJrelXtK+CShbo+KOY8QyygETMDOLqlYFUWEDjsK0Gc4TQFh1XaaawNIq+1kepx5DHU4+ODL0+qLfuZGrd5rDygrfNqx1U3q72j+2l2M4iPH6cgcZk2ydxCKjPLybctUK73xCFdeEfTtwWKklES/BOGss+bRFpSaBfY71SldEZi08fd/FRIQh9bXI5DxaV8mIgEehsQR4TUMkjJfpHRdQwJeEti7hq4isMki3wAUsHTEZ2PJuvrvB5toFEOoJR7Qf9sxERlRIA

//C:\Users\Anuj\AppData\Local\Android\sdk\platform-tools>
//adb location
@Disabled
public class Auto2017Methods extends LinearOpMode {

    DcMotor leftFront = null;  //HAS ENCODER
    DcMotor rightFront = null; //HAS ENCODER
    DcMotor leftBack = null;
    DcMotor rightBack = null;
    DcMotor collection = null;
    DcMotor ScoopUp = null;
    DcMotor delivery = null;
    DcMotor flip = null;
    DcMotor slide = null;
    Servo clawGrab = null;
    Servo clawArm = null;
    Servo stopper = null;
    Servo autoJewel = null;

    //sensors
    ModernRoboticsI2cColorSensor colorR;
    IntegratingGyroscope gyro;
    ModernRoboticsI2cGyro modernRoboticsI2cGyro;
    ColorSensor line;
    BNO055IMU imu;
    ModernRoboticsI2cRangeSensor rangeA;
    ModernRoboticsI2cRangeSensor rangeB;
    //I2cDeviceSynch rangeAreader;


    //variables
    private double circum = 4 * Math.PI;
    private double gearRatio = 45/35;
    private double encoderTicksPerRev = 1120;

    boolean reversed = false, check = false, open = true, scorePos = false, scoring = true;
    int zero = 0;

    byte[] rangeAcache;

    Orientation Pos;
    VuforiaLocalizer vuforia;
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;
    //DO NOT DO RUN TO POSITION

    //reset encoders
    public void initRobot() throws InterruptedException {
        telemetry.addData("Status", "Initialized");

        leftFront  = hardwareMap.get(DcMotor.class, "ld1");
        rightFront = hardwareMap.get(DcMotor.class, "rd1");
        leftBack  = hardwareMap.get(DcMotor.class, "ld2");
        rightBack = hardwareMap.get(DcMotor.class, "rd2");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        collection = hardwareMap.get(DcMotor.class, "clt");
        ScoopUp = hardwareMap.get(DcMotor.class, "Scup");

        delivery = hardwareMap.get(DcMotor.class, "dlv");
        flip = hardwareMap.get(DcMotor.class, "flp");
        slide = hardwareMap.get(DcMotor.class, "sld");
        clawArm = hardwareMap.get(Servo.class, "arm");
        clawGrab = hardwareMap.get(Servo.class, "grab");
        stopper = hardwareMap.get(Servo.class, "stp");
        autoJewel = hardwareMap.get(Servo.class, "auto");


        colorR = (ModernRoboticsI2cColorSensor) hardwareMap.get("colorR");
        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = modernRoboticsI2cGyro;

        line = hardwareMap.colorSensor.get("line");

        line.enableLed(true);
        //rangeA = hardwareMap.i2cDevice.get("range");

        //rangeAreader = new I2cDeviceSynchImpl(rangeA, I2cAddr.create8bit(0x28), false);

        //rangeAreader.engage();


        //********IMU********\\
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //Vuforia
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters();
        params.vuforiaLicenseKey = "AT1IefX/////AAAAGe7qBWfq2E2BmnY2RCELAlIGMKLOUEEXYPeKxkKghQTsCPGGs0xtZTnLSPrE6ExcuJ6WcUeXawPMLqGyDWP9a1YhinvHIaS+8cXJrelXtK+CShbo+KOY8QyygETMDOLqlYFUWEDjsK0Gc4TQFh1XaaawNIq+1kepx5DHU4+ODL0+qLfuZGrd5rDygrfNqx1U3q72j+2l2M4iPH6cgcZk2ydxCKjPLybctUK73xCFdeEfTtwWKklES/BOGss+bRFpSaBfY71SldEZi08fd/FRIQh9bXI5DxaV8mIgEehsQR4TUMkjJfpHRdQwJeEti7hq4isMki3wAUsHTEZ2PJuvrvB5toFEOoJR7Qf9sxERlRIA";
        params.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(params);

        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary



        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        delivery.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flip.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        collection.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        clawGrab.setPosition(.9);
        stopper.setPosition(.75);
        autoJewel.setPosition(.15);
    }

    public void encoderReset() throws InterruptedException {
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(10);
    }

    public void runWithoutEncoders() throws InterruptedException {
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sleep(10);
    }

    public void runUsingEncoders() throws InterruptedException {
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(10);
    }

    //forward
    public void setPower(double power) {
        rightFront.setPower(power);
        rightBack.setPower(power);
        leftFront.setPower(power);
        leftBack.setPower(power);
    }

    public void setPowerSides(double right, double left) {
        rightFront.setPower(right);
        rightBack.setPower(right);
        leftFront.setPower(left);
        leftBack.setPower(left);
    }

    //stop motors
    public void halt() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    /**
     * @param distance INCHES
     * @param power    Percent Power
     * @throws InterruptedException
     */
    public void moveForwardEncoders(double distance, double power) throws InterruptedException { //**********************BACKWARD******************
        encoderReset();
        runWithoutEncoders();
        //sleep(150);

        double counts = ((distance / (gearRatio * circum))) * encoderTicksPerRev;
        double target = rightFront.getCurrentPosition() + counts;


        while (Math.abs(rightFront.getCurrentPosition() - target) > 45 && !isStopRequested()) {
            telemetry.addData("Motor Right 1", rightFront.getCurrentPosition());
            telemetry.addData("Target", target);
            telemetry.update();
            //power = Range.clip(Math.abs(rightFront.getCurrentPosition() - target) / 350, .2, power);
            setPower(-power);
            //idle();

        }

        halt();
    }

    /**
     * @param distance INCHES
     * @param power    Percent Power
     * @throws InterruptedException
     */
    public void moveBackwardEncoders(double distance, double power) throws InterruptedException { //POSITIVE DISTANCE ********FORWARD*****************
        encoderReset();
        runWithoutEncoders();
        //sleep(100);

        double counts = (-distance / (gearRatio * circum)) * encoderTicksPerRev;
        double target = rightFront.getCurrentPosition() + counts;

        setPower(power);
        while (Math.abs(rightFront.getCurrentPosition() - target) > 45 && !isStopRequested()) {
            telemetry.addData("Motor Right 1", rightFront.getCurrentPosition());
            telemetry.addData("Target", target);
            telemetry.update();
            //idle();
        }
        halt();
    }

    public void turnLeft(int target, double power) throws InterruptedException {
        float initial = Pos.firstAngle;
        float current = initial;

        while(Math.abs((initial + current) - target) > 4) {
            current = Pos.firstAngle;

            power = Range.clip((Math.abs((initial + current) - target)), .3, .6);
            setPowerSides(power, -power);
        }

        halt();
    }

    public void turnRight(int target, double power) throws InterruptedException {
        Pos = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float initial = Pos.firstAngle;
        float current = initial;
        target = -(target);

        while(Math.abs((initial + current) - target) > 2 && opModeIsActive()) {
            Pos = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            current = Pos.firstAngle; //getHeading
            telemetry.addData("toGo", current - target);
            telemetry.update();
            power = Range.clip((Math.abs(current - target)), power, .6);
            setPowerSides(-power, power);
            //idle();
        }

        halt();
    }

    public void turn90Encoders() throws InterruptedException {
        double initR = rightFront.getCurrentPosition()
                , initL = leftFront.getCurrentPosition();

        boolean right = true, left = true;
        int value = 850;

        runWithoutEncoders();
        while (opModeIsActive() && (right || left)) {
            setPowerSides(-.4, .4);

            if(Math.abs(rightFront.getCurrentPosition()) > (value+initR)) {
                right = false;
            }

            if(Math.abs(leftFront.getCurrentPosition()) > ((value)+initL)) {
                left = false;
            }
        }
        halt();
    }
    public void turn90EncodersL() throws InterruptedException {
        double initR = rightFront.getCurrentPosition()
                , initL = leftFront.getCurrentPosition();

        boolean right = true, left = true;
        int value = 850;

        runWithoutEncoders();
        while (opModeIsActive() && (right || left)) {
            setPowerSides(.4, -.4);

            if(Math.abs(rightFront.getCurrentPosition()) > (value+initR)) {
                right = false;
            }

            if(Math.abs(leftFront.getCurrentPosition()) > ((value)+initL)) {
                left = false;
            }
        }
        halt();
    }

    /*public double getUSValue(int expected) {
        int count2 = 0;
        while (count2 < 10 && opModeIsActive()) {
            rangeAcache = rangeAreader.read(0x04, 1);
            int LUS = rangeAcache[0] & 0xFF;

            if (Math.abs(LUS - expected) < 100) {
                return LUS;
            }
            count2++;
        }
        return 255;
    }*/



    //score red Jewel
    public void scoreJewelRed () throws InterruptedException {
        if (null != hardwareMap) {
            //jewelArm.setDirection(Servo.Direction.FORWARD);
            if (null != colorR && null != autoJewel) {
                // move jewelArm from current position to near jewels (red/blue); need to find out position
                for(int i=0; i <= 50; i++) {
                    autoJewel.setPosition(.2+(.01*i));
                    sleep(10);
                }
                autoJewel.setPosition(.76);
                colorR.enableLed(true);
                sleep(500);

                // scan the color using colorSensor
                int colDif = colorR.blue() - colorR.red();
                sleep(500);
                int i = 0;
                while(Math.abs(colDif) < 1 && i < 7) {
                    autoJewel.setPosition(.76-(.01*i));

                    colDif = colorR.blue() - colorR.red();
                    sleep(500);
                    telemetry.addData("B-R", colDif);
                    telemetry.update();
                    i++;
                }


                if (colDif > 1) {
                    // in case of red move forward
                    telemetry.addData("ball:", "blue");
                    telemetry.update();
                    sleep(1000);
                    setPower(.3);
                    sleep(200);
                    halt();
                    sleep(100);
                    autoJewel.setPosition(.2);
                    sleep(5000);

                    moveForwardEncoders(12, .4);

                    //autoJewel.setPosition(.2);
                } else if (colDif < -1) {

                    telemetry.addData("ball:", "red");
                    telemetry.update();
                    sleep(1000);
                    setPower(-.3);
                    sleep(300);
                    halt();
                    sleep(100);

                    autoJewel.setPosition(.2);
                    sleep(5000);
                    //autoJewel.setPosition(.2);
                    moveForwardEncoders(20, .4);
                }
                else {
                    autoJewel.setPosition(.2);
                    sleep(5000);

                    moveForwardEncoders(13, .4);
                }
            }
        }
    }

    //score blue Jewel
    public void scoreJewelBlue () throws InterruptedException {
        if (null != hardwareMap) {
            //jewelArm.setDirection(Servo.Direction.FORWARD);
            if (null != colorR && null != autoJewel) {
                // move jewelArm from current position to near jewels (red/blue); need to find out position
                for(int i=0; i <= 50; i++) {
                    autoJewel.setPosition(.2+(.01*i));
                    sleep(10);
                }
                autoJewel.setPosition(.76);
                colorR.enableLed(true);
                sleep(500);

                // scan the color using colorSensor
                int colDif = colorR.blue() - colorR.red();
                sleep(500);
                int i = 0;
                while(Math.abs(colDif) < 1 && i < 7) {
                    autoJewel.setPosition(.76-(.01*i));

                    colDif = colorR.blue() - colorR.red();
                    sleep(500);
                    telemetry.addData("B-R", colDif);
                    telemetry.update();
                    i++;
                }

                if (colDif > 1) {
                    telemetry.addData("ball:", "blue");
                    telemetry.update();
                    //sleep(1000);
                    moveForwardEncoders(3, .2);
                    halt();
                    sleep(100);
                    autoJewel.setPosition(.2);
                    sleep(1000);
                    //autoJewel.setPosition(.2);

                    moveBackwardEncoders(20, .35);

                } else if (colDif < -1) {
                    // in case of red move backwards
                    telemetry.addData("ball:", "red");
                    telemetry.update();
                    sleep(1000);
                    moveBackwardEncoders(3, .2);
                    halt();
                    sleep(100);

                    autoJewel.setPosition(.2);
                    sleep(1000);

                    moveBackwardEncoders(12, .4);
                } else {
                    autoJewel.setPosition(.2);
                    sleep(1000);

                    moveBackwardEncoders(13, .4);
                }

            }
        }
    }

    public void jewelSweep() throws InterruptedException {

        int jewelB = colorR.blue();
        int jewelR = colorR.red();

        while(jewelB > 4 && jewelR < 2) {
            for (int i = 0; i <=5; i++)
                autoJewel.setPosition(.2+(.01*i));
            sleep(10);
            autoJewel.setPosition(.1);
            sleep(10);
        }

    }



    public int vuScoreia (double timeout) {
        double timeI = getRuntime();
        while(opModeIsActive()) {
            if(getRuntime() > timeI + timeout) {
                break;
            }
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                switch  (vuMark) {
                    case LEFT:
                        telemetry.addData("Score", "Left Col");
                        return 0;
                    case CENTER:
                        telemetry.addData("Score", "Center Col");
                        return 1;
                    case RIGHT:
                        telemetry.addData("Score", "Right Col");
                        return 2;
                }
                telemetry.addData("seen", vuMark);
            }
            else {
                telemetry.addData("VuMark", "not visible");
            }
            telemetry.update();
        }
        return 0;
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public double motorPower(double joy1) {
        double[] values = {0.0, 0.15, 0.35, 0.55, 0.65, 0.85, 1.0};
        double sign = joy1/Math.abs(joy1);

        if(Math.abs(joy1) < values[1]) {
            return 0;
        } else if(Math.abs(joy1) < values[2]) {
            return sign*values[1];
        } else if(Math.abs(joy1) < values[3]) {
            return sign*values[2];
        } else if(Math.abs(joy1) < values[4]) {
            return sign*values[3];
        } else if(Math.abs(joy1) < values[5]) {
            return sign*values[4];
        } else if(Math.abs(joy1) < values[6]) {
            return sign*values[5];
        } else {
            return sign*1.0;
        }
    }

    public void Auto2017Methods() {
    }

    public void runOpMode() throws InterruptedException {

    }
}
