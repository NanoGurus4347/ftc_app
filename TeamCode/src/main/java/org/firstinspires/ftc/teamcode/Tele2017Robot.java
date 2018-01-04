/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="TeleOp", group="Iterative Opmode")
//@Disabled
public class Tele2017Robot extends OpMode {
    // Declare OpMode members.
    //private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private DcMotor collection = null;
    private DcMotor delivery = null;
    private DcMotor flip = null;
    private DcMotor slide = null;
    private Servo clawGrab = null;
    private Servo clawArm = null;
    private Servo stopper = null;
    private Servo autoJewel = null;
    boolean reversed = false, check = false, open = true, scorePos = false, scoring = true;
    int zero = 0;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        leftFront  = hardwareMap.get(DcMotor.class, "ld1");
        rightFront = hardwareMap.get(DcMotor.class, "rd1");
        leftBack  = hardwareMap.get(DcMotor.class, "ld2");
        rightBack = hardwareMap.get(DcMotor.class, "rd2");
        collection = hardwareMap.get(DcMotor.class, "clt");
        delivery = hardwareMap.get(DcMotor.class, "dlv");
        flip = hardwareMap.get(DcMotor.class, "flp");
        slide = hardwareMap.get(DcMotor.class, "sld");
        clawArm = hardwareMap.get(Servo.class, "arm");
        clawGrab = hardwareMap.get(Servo.class, "grab");
        stopper = hardwareMap.get(Servo.class, "stp");
        autoJewel = hardwareMap.get(Servo.class, "auto");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        delivery.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flip.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        collection.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        clawGrab.setPosition(.9);
        stopper.setPosition(.75);
        autoJewel.setPosition(.2);

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        //runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double left   = motorPower(gamepad1.left_stick_y);
        double right  = motorPower(gamepad1.right_stick_y);
        //imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        telemetry.addData("mR1", rightFront.getCurrentPosition());
        telemetry.addData("mR2", rightBack.getCurrentPosition());
        telemetry.addData("mL1", leftFront.getCurrentPosition());
        telemetry.addData("mL2", leftBack.getCurrentPosition());
        autoJewel.setPosition(.2);

        if (reversed) {
            if (gamepad1.right_stick_button) {
                rightFront.setPower(right / 3);
                leftFront.setPower(left / 3);
                rightBack.setPower(right / 3);
                leftBack.setPower(left / 3);
            } else {
                rightFront.setPower(right);
                leftFront.setPower(left);
                rightBack.setPower(right);
                leftBack.setPower(left);
            }
        } else if (!reversed) {
            if (gamepad1.right_stick_button) {
                rightFront.setPower(-left / 3);
                leftFront.setPower(-right / 3);
                rightBack.setPower(-left / 3);
                leftBack.setPower(-right / 3);
            } else {
                rightFront.setPower(-left);
                leftFront.setPower(-right);
                rightBack.setPower(-left);
                leftBack.setPower(-right);
            }
        }

        if (gamepad1.dpad_left) {
            check = true;
        }

        if(check && (!gamepad1.dpad_left)) {
            reversed = !reversed;
            check = false;
        }

        if(gamepad1.right_trigger > .05) {
            collection.setPower(-gamepad1.right_trigger/1.25);
        } else if(gamepad1.left_trigger > .05) {
            collection.setPower(gamepad1.left_trigger/1.25);
        } else {
            collection.setPower(0);
        }

        if(gamepad1.dpad_up) {
            stopper.setPosition(.75);
        } else if(gamepad1.dpad_down) {
            stopper.setPosition(.4);
        }

        if(gamepad1.b) {
            clawGrab.setPosition(.75);
        } else if(gamepad1.y) {
            clawGrab.setPosition(.3);
        }

        if(gamepad1.left_bumper) {
            clawArm.setPosition(.3);
        } else if (gamepad1.x) {
            clawArm.setPosition(.875);
        } else if(gamepad1.right_bumper) {
            clawArm.setPosition(0.135);
        }

        if(gamepad1.start) {
            zero = delivery.getCurrentPosition();
        }

        if(gamepad2.b) {
            delivery.setPower(.35);
            scorePos = false;
        } else if(gamepad2.a && gamepad2.x) {
            scorePos = true;//-440!!!
        } else if(gamepad2.x) {
            delivery.setPower(-.75);
            //scorePos = false;
        } else if(!scorePos){
            delivery.setPower(0.0); //set position instead
        }

        if(scorePos && delivery.getCurrentPosition() > zero + -440*3) {
            delivery.setPower(-.75);
            flip.setPower(-.2);
        } else {
            //scoring = false;
            scorePos = false;
            //delivery.setPower(0.0);
        }

//        if(gamepad2.b) {
//            delivery.setPower(.35);
//            scoring = false;
//        } else if(gamepad2.a && gamepad2.x) {
//            scorePos = true;//-440!!!
//        } else if(gamepad2.x) {
//            delivery.setPower(-.75);
//        } else if(!scorePos && !scoring){
//            delivery.setPower(0);
//        } else if(scoring) {
//            delivery.setPower(-.2); //set position instead
//        }
//
//        if(scorePos && delivery.getCurrentPosition() > zero + -450) {
//            delivery.setPower(-.75);
//            flip.setPower(-.2);
//        } else if(Math.abs(delivery.getCurrentPosition() - (zero + -450)) < 10) {
//            scorePos = false;
//            scoring = true;
//        } else {
//            scoring = false;
//            scorePos = false;
//        }


        telemetry.addData("dlv", delivery.getCurrentPosition());

        if(gamepad2.right_bumper) {
            flip.setPower(.4);
        } else if(gamepad2.left_trigger > .3) {
            flip.setPower(-gamepad2.left_trigger/2);
        } else {
            flip.setPower(0);
        }

        if(gamepad2.right_trigger > .05) {
            slide.setPower(gamepad2.right_trigger);
        } else if(gamepad2.left_bumper) {
            slide.setPower(-0.5);
        } else {
            slide.setPower(0);
        }


        // Show the elapsed game time and wheel power.
        telemetry.update();
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
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}

