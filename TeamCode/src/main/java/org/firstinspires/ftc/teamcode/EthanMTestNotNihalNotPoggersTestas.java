package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.lang.Math;
import java.math.BigDecimal;

@TeleOp

public class EthanMTestNotNihalNotPoggersTestas extends LinearOpMode {
    private Blinker expansion_Hub_1;
    private DcMotor BigMotor;
    private CRServo ServoPower;
    private double LeftStickValue;
    @Override
    public void runOpMode() {
        expansion_Hub_1 = hardwareMap.get(Blinker.class, "Nihal");
        BigMotor = hardwareMap.get(DcMotor.class, "Big Motor");
        ServoPower = hardwareMap.get(CRServo.class,"ServoPower");
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("left stick y axis", this.gamepad1.left_stick_y);
            telemetry.addData("right stick y axis", this.gamepad1.right_stick_y);
            telemetry.update();
            LeftStickValue = this.gamepad1.left_stick_y;
            if (LeftStickValue == 0) {
                BigMotor.setPower(0.1);
            }
            else {
                BigMotor.setPower(LeftStickValue*0.5);
            }
            ServoPower.setPower(this.gamepad1.right_stick_y);

        }
    }
}