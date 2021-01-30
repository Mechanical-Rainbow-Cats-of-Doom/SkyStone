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
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.lang.Math;  //This is the standard Java package for a variety of math functions

@TeleOp

public class ChassisMovementCode extends LinearOpMode {

    private Blinker Control_Hub;

    //motors
    private DcMotor back_right_wheel;
    private DcMotor front_right_wheel;
    private DcMotor back_left_wheel;
    private DcMotor front_left_wheel;

    private BNO055IMU imu;

    private DigitalChannel switch_;

    private class Chassis {
        double frontLeft;
        double frontRight;
        double backLeft;
        double backRight;
        double rightEncoder;
        double leftEncoder;
        double backEncoder;
        double clearRight = 0;
        double clearLeft = 0;
        double clearBack = 0;
        double fieldLength = 141;
        double robotLength = 17.25;
        double robotWidth = 17.375;
        double countsPerRotation = 360;
        double trueDrive;
        double drivePreset;

        private void SetAxisMovement () {
            trueDrive = (rightEncoder+leftEncoder)/2;
        }

        private void Forward (double forwardLength) {
            drivePreset = trueDrive;
            while (drivePreset > trueDrive + forwardLength) {
                front_right_wheel.setPower(50);
                front_left_wheel.setPower(50);
                back_left_wheel.setPower(50);
                back_right_wheel.setPower(50);
                trueDrive = (rightEncoder+leftEncoder)/2;
            }
        }

        private void ZeroEncoders () {
            clearRight = back_right_wheel.getCurrentPosition()/360*1.173150521;
            clearLeft = -front_right_wheel.getCurrentPosition()/360*1.178221633;
            clearBack = front_left_wheel.getCurrentPosition()/360*1.17584979;
        }

        private void Encoders () {
            rightEncoder = back_right_wheel.getCurrentPosition()/360*1.173150521-clearRight;
            leftEncoder = -front_right_wheel.getCurrentPosition()/360*1.178221633-clearLeft;
            backEncoder = front_left_wheel.getCurrentPosition()/360*1.17584979-clearBack;
            telemetry.addData("True back", backEncoder/360*1.17584979);
            telemetry.addData("True right", rightEncoder/360*1.173150521);
            telemetry.addData("True left", leftEncoder/360*1.178221633);
            telemetry.addData("Right Encoder CM", rightEncoder);
            telemetry.addData("Left Encoder CM", leftEncoder);
            telemetry.addData("Back Encoder CM", backEncoder);
            telemetry.addData("Right Encoder",back_right_wheel.getCurrentPosition());
            telemetry.addData("Left Encoder",front_right_wheel.getCurrentPosition());
            telemetry.addData("Back Encoder",front_left_wheel.getCurrentPosition());
            telemetry.update();
        }

        private void SetMotors (double drive, double strafe, double rotate) {
            this.frontLeft = drive - strafe - rotate;
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

    enum OperState {
        NORMALDRIVE,
        NORMALROTATE,
        FORWARD
    }

    @Override
    public void runOpMode(){
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        Control_Hub = hardwareMap.get(Blinker.class, "Control Hub");

        front_left_wheel = hardwareMap.get(DcMotor.class, "front left wheel");
        front_right_wheel = hardwareMap.get(DcMotor.class, "front right wheel");
        back_left_wheel = hardwareMap.get(DcMotor.class, "back left wheel");
        back_right_wheel = hardwareMap.get(DcMotor.class, "back right wheel");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();  //in wrong spot--where is better?
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        imu.initialize(parameters);

        waitForStart();

        double drive;
        double strafe;
        double rotate;
        ChassisMovementCode.Chassis chasty = new ChassisMovementCode.Chassis();
        ChassisMovementCode.OperState driveOpState = ChassisMovementCode.OperState.NORMALDRIVE;

        while (opModeIsActive()) {

            switch (driveOpState) {
                case NORMALDRIVE:
                    drive = -this.gamepad1.left_stick_y;
                    strafe = -this.gamepad1.left_stick_x;
                    rotate = 0;

                    chasty.SetMotors (drive, strafe, rotate);
                    chasty.Drive();
                    chasty.Encoders();
                    chasty.SetAxisMovement();

                    if (this.gamepad1.left_trigger != 0) {
                        chasty.ZeroEncoders();
                    }

                    if (this.gamepad1.right_trigger != 0) {
                        driveOpState = ChassisMovementCode.OperState.NORMALROTATE;
                    }

                    if (this.gamepad1.a) {
                        driveOpState = ChassisMovementCode.OperState.FORWARD;
                    }

                    break;

                case NORMALROTATE:
                    if (this.gamepad1.right_trigger != 0) {
                        rotate = -this.gamepad1.right_stick_x;
                        drive = 0;
                        strafe = 0;

                        chasty.SetMotors(drive, strafe, rotate);
                        chasty.Drive();
                        chasty.Encoders();
                        chasty.SetAxisMovement();

                        if (this.gamepad1.left_trigger != 0) {
                            chasty.ZeroEncoders();
                        }
                    }
                    else {
                        driveOpState = ChassisMovementCode.OperState.NORMALDRIVE;
                    }
                    break;

                case FORWARD:

                    double forwardLength = 50;
                    chasty.SetAxisMovement();
                    chasty.Forward(forwardLength);

                    driveOpState = ChassisMovementCode.OperState.NORMALDRIVE;

                    break;

                default :
                    break;
            }
        }
    }
}
