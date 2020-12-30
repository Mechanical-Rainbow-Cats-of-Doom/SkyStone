package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
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

public class EthanTestCode extends LinearOpMode {
    //declarations of everything
    private Blinker expansion_Hub_1;
    private Blinker expansion_Hub_2;
    private Servo back_foundation_puller;
    private DcMotor back_left_wheel;
    private DcMotor back_right_wheel;
    private DcMotor front_left_wheel;
    private DcMotor front_right_wheel;
    private DistanceSensor color_sensor;
    private DistanceSensor distance_sensor;
    private ColorSensor front_color_sensor;
    private BNO055IMU imu;
    private DcMotor lift_motor;
    private DcMotor clamp_motor;
    private DigitalChannel switch_;

    public EthanTestCode() {
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }

}
