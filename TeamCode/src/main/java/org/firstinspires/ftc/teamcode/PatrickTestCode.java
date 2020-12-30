package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;  //This is the package for controlling the IMU
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.lang.Math;  //This is the standard Java package for a variety of math functions

@TeleOp

public class PatrickTestCode extends LinearOpMode{
    private Blinker expansion_Hub_1;
    private Blinker expansion_Hub_2;
    //Declaration for the servo for the puller
    private Servo back_foundation_puller;
    //Declaration for the four drive motors
    private DcMotor back_left_wheel;
    private DcMotor back_right_wheel;
    private DcMotor front_left_wheel;
    private DcMotor front_right_wheel;
    //Declaration for the sensors.  These classes were created to work with the default/supported
    //sensors on the robot.  Technically, if we ever run into a sensor that has the same interface
    //as one of the ports on the Expansion Hub, you could design your own interface.
    private DistanceSensor color_sensor;
    private DistanceSensor distance_sensor;
    private ColorSensor front_color_sensor;
    //Declaration for the Inertial Measurement Unit (IMU)--note:  This is the built in gyroscope
    //in the Expansion Hub.
    private BNO055IMU imu;
    private DcMotor lift_motor;
    private DcMotor clamp_motor;
    //Below is the declaration for the digital "red/blue" switch we have on our old robot.
    //The system has a collection of other interfaces that can be used regularly.
    //NOTE:  For those of you using motion control/needing start/stop locations, you could use
    //something like this for a limit switch or something--and then use it as part of a closed loop
    //control algorithm.
    private DigitalChannel switch_;


    private class Chassis {
        double frontLeft;
        double frontRight;
        double backLeft;
        double backRight;


        private void SetMotors (double drive, double strafe, double rotate) {
            this.frontLeft = -drive + strafe + rotate;
            this.backLeft = -drive - strafe + rotate;
            this.frontRight = drive + strafe + rotate;
            this.backRight = drive - strafe + rotate;
        }


        private void Drive () {
            front_right_wheel.setPower(this.frontRight);
            front_left_wheel.setPower(this.frontLeft);
            back_left_wheel.setPower(this.backLeft);
            back_right_wheel.setPower(this.backRight);
        }
    }
}
