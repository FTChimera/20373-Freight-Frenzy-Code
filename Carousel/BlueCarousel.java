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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Autonomous(name="BlueCarousel", group="Linear Opmode")
@Disabled
public class BlueCarousel extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime ctimer = new ElapsedTime();
    private DcMotor motorFrontLeft = null;
    private DcMotor motorBackLeft = null;
    private DcMotor motorFrontRight = null; 
    private DcMotor motorBackRight = null;
    private Servo Grip = null;
    private Servo Grip2 = null;
    private CRServo ArmTower = null;
    private DcMotor Arm = null;
    private DcMotor carousel = null;
    static final double HD_COUNTS_PER_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = 20.15293;
    static final double WHEEL_CIRCUMFERENCE_MM = 90 * Math.PI;
    static final double DRIVE_COUNTS_PER_MM = (HD_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_MM;
    static final double DRIVE_COUNTS_PER_IN = DRIVE_COUNTS_PER_MM * 25.4;

    private void Drive(int txsfl, int txsfr, int txsbl, int txsbr) {
        int fl = motorFrontLeft.getCurrentPosition()+(int)(txsfl*DRIVE_COUNTS_PER_IN) * -1;
        motorFrontLeft.setTargetPosition(fl);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontLeft.setPower(0.4);
        int br = motorBackRight.getCurrentPosition()+(int)(txsbr*DRIVE_COUNTS_PER_IN) *-1;
        motorBackRight.setTargetPosition(br);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setPower(0.4);
        
        int fr = motorFrontRight.getCurrentPosition()+(int)(txsfr*DRIVE_COUNTS_PER_IN)*-1;
        motorFrontRight.setTargetPosition(fr);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setPower(0.4);
        int bl = motorBackLeft.getCurrentPosition()+(int)(txsbl*DRIVE_COUNTS_PER_IN)*-1;
        motorBackLeft.setTargetPosition(bl);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setPower(0.4);
        while (motorBackLeft.isBusy() && opModeIsActive()){
        }
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        motorFrontRight  = hardwareMap.get(DcMotor.class, "motorFrontRight");
        motorFrontLeft  = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motorBackRight  = hardwareMap.get(DcMotor.class, "motorBackRight");
        motorBackLeft  = hardwareMap.get(DcMotor.class, "motorBackLeft");
        carousel = hardwareMap.dcMotor.get("CarouselSpinner");
        ArmTower = hardwareMap.crservo.get("ArmTower");
        Arm = hardwareMap.dcMotor.get("Arm");
        Grip = hardwareMap.servo.get("Grip");
        Grip2 = hardwareMap.servo.get("Grip2");
        Grip.setPosition(-1);
        Grip2.setPosition(1);
        // Wait for the game to start (driver presses PLAY)
        
        
        waitForStart();
        runtime.reset();
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        
        
        // run until the end of the match (driver presses STOP)
        if (opModeIsActive()) {
            //Arm.setTargetPosition(138);
            //Arm.setTargetPosition(275);
            Arm.setTargetPosition(475);
            Arm.setPower(0.4);
            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (Arm.isBusy()) {
            }
        
            Drive(-18, -18, -18, -18);
            carousel.setPower(0.7);
            ctimer.reset();
            
            while (8>ctimer.seconds()) {
            }
            carousel.setPower(0);
            
            Drive(34, -34, -34, 34);
            Drive(30, 30, 30, 30);
            Grip.setPosition(1);
            Grip2.setPosition(-1);
            
            timer.reset();
            while (2>timer.seconds()) {
            }
            Arm.setTargetPosition(550);
            Arm.setPower(0.4);
            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (Arm.isBusy()) {
            }
            Drive(-49, -49, -49, -49);
            Drive(-6, 6, 6, -6);
            Arm.setTargetPosition(0);
            Arm.setPower(0.4);
            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (Arm.isBusy()) {
            }
            telemetry.update(); 
        }
    }
}
