package org.firstinspires.ftc.teamcode;
import android.os.DropBoxManager;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import java.lang.Math;
import java.lang.reflect.Array;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
@TeleOp

public class DistanceSensorTesting extends LinearOpMode {
    private DistanceSensor DistanceSensor;

    @Override
    public void runOpMode() {
        DistanceSensor = hardwareMap.get(DistanceSensor.class, "Distance Sensor");
        double AveragedArray;
        int i;
        double total = 0;
        int index = 0;
        int ArraySize = 10;
        double [] sensorArray;
        sensorArray = new double[ArraySize];
        waitForStart();
        while (opModeIsActive()) {
            sensorArray[index] = DistanceSensor.getDistance(DistanceUnit.MM);
            if (index >= (ArraySize - 1)) {
                index = 0;
            }
            else {
                index++;
            }
            for (i = 0; i<sensorArray.length; i++) {
                total = total + sensorArray[i];
            }
            AveragedArray = total / sensorArray.length;
            telemetry.addData("Average Array", AveragedArray);
            telemetry.update();
        }
    }
}