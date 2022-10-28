package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="FCMecanum", group="TeleOp")
public class FieldCentricMecanum extends LinearOpMode {
    private DcMotor motorFrontLeft = null;
    private DcMotor motorFrontRight = null;
    private DcMotor motorBackLeft = null;
    private DcMotor motorBackRight = null;
    private final double driveAdjuster = 1;

    BNO055IMU imu;
    Orientation angles;

    @Override
    public void runOpMode()  throws InterruptedException {

        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag     = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.update();
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();
        
        while (opModeIsActive()) {
        
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            final double globalAngle = angles.firstAngle;

            //Finds the hypotenous of the triangle created by the two joystick values. Used to find the absoulte direction to go in.
            final double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            //Finds the robot's angle from the raw values of the joystick
            final double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI /4;
            final double rightX = gamepad1.right_stick_x;

            double v1 = r * Math.sin(robotAngle - globalAngle/57) - rightX;
            double v2 = r * Math.cos(robotAngle - globalAngle/57) + rightX;
            double v3 = r * Math.cos(robotAngle - globalAngle/57) - rightX;
            double v4 = r * Math.sin(robotAngle - globalAngle/57) + rightX;

            if (Math.abs(v1) > 1 || Math.abs(v2) > 1 || Math.abs(v3) > 1 || Math.abs(v4) > 1 ) {
                // Find the largest power
                double max = 0;
                max = Math.max(Math.abs(v1), Math.abs(v2));
                max = Math.max(Math.abs(v3), max);
                max = Math.max(Math.abs(v4), max);

                // Divide everything by max (it's positive so we don't need to worry
                // about signs)
                v1 /= max;
                v2 /= max;
                v3 /= max;
                v4 /= max;
            }

            motorFrontRight.setPower(v1);
            motorFrontLeft.setPower(v2);
            motorBackRight.setPower(v3);
            motorBackLeft.setPower(v4);

            telemetry.addData("Heading ", globalAngle);
            telemetry.addData("Stick ", robotAngle);
            telemetry.update();
        }
    }
}
