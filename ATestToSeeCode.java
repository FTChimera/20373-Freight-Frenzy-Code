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
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains a minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
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

@TeleOp(name="TeleOp", group="Linear Opmode")

public class ATestToSeeCode extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorFrontLeft = null;
    private DcMotor motorBackLeft = null;
    private DcMotor motorFrontRight = null; 
    private DcMotor motorBackRight = null;
    private Servo Grip = null;
    private Servo Grip2 = null;
    
    private DcMotor ArmTower = null;
    private DcMotor Arm = null;
    private DcMotor carousel = null;
    BNO055IMU imu;
    Orientation angles;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        motorFrontRight  = hardwareMap.get(DcMotor.class, "motorFrontRight");
        motorFrontLeft  = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motorBackRight  = hardwareMap.get(DcMotor.class, "motorBackRight");
        motorBackLeft  = hardwareMap.get(DcMotor.class, "motorBackLeft");
        carousel = hardwareMap.dcMotor.get("CarouselSpinner");
        ArmTower = hardwareMap.dcMotor.get("ArmTower");
        Arm = hardwareMap.dcMotor.get("Arm");
        Grip = hardwareMap.servo.get("Grip");
        Grip2 = hardwareMap.servo.get("Grip2");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        Arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        double cur_angle = 0;
        
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
          angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
          double globalAngle = angles.firstAngle;
          telemetry.addData("Heading", angles.firstAngle);
          telemetry.addData("Roll", angles.secondAngle);
          telemetry.addData("Pitch", angles.thirdAngle);
          double y = gamepad1.left_stick_y;
          double x = -gamepad1.left_stick_x*1.1;
          double rx = gamepad1.right_stick_x;
          double ty = gamepad2.left_stick_y;
          double tx = -gamepad2.left_stick_x*1.1;
          double trx = gamepad2.right_stick_x;
          double denominator = (Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1)*1.5);
          
          if (ty != 0 || tx != 0 || trx != 0) {
            y = ty;
            x= tx;
            rx=trx;
            double a_number = 4;
            if (gamepad2.right_stick_button) {
              a_number = 1.5;
            }
            denominator = (Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1)*a_number);
          }
          if (gamepad1.left_bumper || ty != 0 || tx != 0 || trx != 0) {
            double frontLeftPower = (y + x - rx) / (denominator);
            double backLeftPower = (y - x - rx) / (denominator);
            double frontRightPower = (y - x + rx) / (denominator);
            double backRightPower = (y + x + rx) / (denominator);
            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);
            
          }
          else {
            double globalAngl = globalAngle-cur_angle;
            //Finds the hypotenous of the triangle created by the two joystick values. Used to find the absoulte direction to go in.
            final double r = Math.hypot(gamepad1.left_stick_x*1.1, gamepad1.left_stick_y);
            //Finds the robot's angle from the raw values of the joystick
            final double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x*1.1) - Math.PI /4;
            final double rightX = gamepad1.right_stick_x;
            denominator = (Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1)*1.5);
            double frontRightPower = r * Math.sin(robotAngle - globalAngl/57) - rightX;
            double frontLeftPower = r * Math.cos(robotAngle - globalAngl/57) + rightX;
            double backRightPower = r * Math.cos(robotAngle - globalAngl/57) - rightX;
            double backLeftPower = r * Math.sin(robotAngle - globalAngl/57) + rightX;
            if (Math.abs(frontRightPower) > 1 || Math.abs(frontLeftPower) > 1 || Math.abs(backRightPower) > 1 || Math.abs(backLeftPower) > 1 ) {
                // Find the largest power
                double max = 0;
                max = Math.max(Math.abs(frontRightPower), Math.abs(frontLeftPower));
                max = Math.max(Math.abs(backRightPower), max);
                max = Math.max(Math.abs(backLeftPower), max);

                // Divide everything by max (it's positive so we don't need to worry
                // about signs)
                frontRightPower /= max;
                frontLeftPower /= max;
                backRightPower /= max;
                backLeftPower /= max;
            }
            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);
          }
          if (gamepad1.right_bumper) {
            cur_angle = globalAngle;
          }
          if(gamepad2.x){
            carousel.setPower(-1);
          }
          else if(gamepad2.b){
            carousel.setPower(1);
          }
          else{
            carousel.setPower(0);
          }
          if (gamepad2.dpad_up) {
            Arm.setPower(0.8);
          } else if (gamepad2.dpad_down) {
            Arm.setPower(-0.6);
          } else {
            Arm.setPower(0);
          }
          if (gamepad2.dpad_left) {
            ArmTower.setPower(-0.45);
          } else if (gamepad2.dpad_right) {
            ArmTower.setPower(0.45);
          } else {
            ArmTower.setPower(0);
          }
          if (gamepad1.right_stick_button) {
            
          }
          else {
            if (gamepad2.left_bumper) {
              Grip.setPosition(1);
              Grip2.setPosition(-1);
            
            } else if (gamepad2.right_bumper) {
              Grip.setPosition(-1);
              Grip2.setPosition(1);
            }
          }
          telemetry.addData("cur_angle", cur_angle);
          telemetry.addData("Arm", Arm.getCurrentPosition());
          telemetry.update();
        }
      }
    }



