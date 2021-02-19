package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;  //This is the package for controlling the IMU
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous
public class firstAutonomousMode extends LinearOpMode {

    private Blinker Control_Hub;

    //motors
    private DcMotor back_right_wheel;
    private DcMotor front_right_wheel;
    private DcMotor back_left_wheel;
    private DcMotor front_left_wheel;

    private BNO055IMU imu;

    private DigitalChannel switch_;

    private class Chassis {

        double rotation;
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
        double trueStrafe;
        double slowIntensity = 10;
        double trueRotate;
        double backRightMultiplier = 1;
        double backLeftMultiplier = 1;
        double frontRightMultiplier = 0.9;
        double frontLeftMultiplier = 0.9;

        private void SetAxisMovement () {
            trueDrive = (rightEncoder+leftEncoder)/2;
            trueStrafe = backEncoder - (rightEncoder-leftEncoder)/2;
            trueRotate = (rightEncoder-leftEncoder)/2;
        }

        private void ForwardAndBackward (double drivePreset) {
            front_right_wheel.setPower(frontRightMultiplier*Math.signum(drivePreset - trueDrive) * Math.max(0.15, Math.abs(drivePreset - trueDrive) / drivePreset));
            front_left_wheel.setPower(frontLeftMultiplier*Math.signum(drivePreset - trueDrive) * Math.max(0.15, Math.abs(drivePreset - trueDrive) / drivePreset));
            back_left_wheel.setPower(-1*backLeftMultiplier*Math.signum(drivePreset - trueDrive) * Math.max(0.15, Math.abs(drivePreset - trueDrive) / drivePreset));
            back_right_wheel.setPower(backRightMultiplier*Math.signum(drivePreset - trueDrive) * Math.max(0.15, Math.abs(drivePreset - trueDrive) / drivePreset));

        }

        private double CorrectRotation (double currentRotation, double rotationGoal) {

            rotation = Math.signum(rotationGoal - currentRotation) * (Math.max(0.2, Math.abs(rotationGoal - currentRotation) / 180));
            return(rotation);
        }

        private void LeftAndRight (double drivePreset) {
            front_right_wheel.setPower(-1*frontRightMultiplier*Math.signum(drivePreset - trueStrafe) * Math.max(0.15, Math.abs(drivePreset - trueStrafe) / drivePreset));
            front_left_wheel.setPower(frontLeftMultiplier*Math.signum(drivePreset - trueStrafe) * Math.max(0.15, Math.abs(drivePreset - trueStrafe) / drivePreset));
            back_left_wheel.setPower(backLeftMultiplier*Math.signum(drivePreset - trueStrafe) * Math.max(0.15, Math.abs(drivePreset - trueStrafe) / drivePreset));
            back_right_wheel.setPower(backRightMultiplier*Math.signum(drivePreset - trueStrafe) * Math.max(0.15, Math.abs(drivePreset - trueStrafe) / drivePreset));

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

        }

        private void Telemetry () {
            telemetry.addData("True back", backEncoder/360*1.17584979);
            telemetry.addData("True right", rightEncoder/360*1.173150521);
            telemetry.addData("True left", leftEncoder/360*1.178221633);
            telemetry.addData("Right Encoder CM", rightEncoder);
            telemetry.addData("Left Encoder CM", leftEncoder);
            telemetry.addData("Back Encoder CM", backEncoder);
            telemetry.addData("Right Encoder",back_right_wheel.getCurrentPosition());
            telemetry.addData("Left Encoder",front_right_wheel.getCurrentPosition());
            telemetry.addData("Back Encoder",front_left_wheel.getCurrentPosition());
            telemetry.addData("Drive",trueDrive);
            telemetry.addData("Strafe", trueStrafe);
            telemetry.addData("Rotate", trueRotate);
            telemetry.update();
        }

        private void SetMotors (double drive, double strafe, double rotate) {
            this.frontLeft = drive - strafe - rotate;
            this.backLeft = -drive - strafe + rotate;
            this.frontRight = drive + strafe + rotate;
            this.backRight = drive - strafe + rotate;
        }


        private void Drive () {
            front_right_wheel.setPower(this.frontRight*frontRightMultiplier);
            front_left_wheel.setPower(this.frontLeft*frontLeftMultiplier);
            back_left_wheel.setPower(this.backLeft*backLeftMultiplier);
            back_right_wheel.setPower(this.backRight*backRightMultiplier);
        }
    }

    enum OperState {
        FIRSTMOVE,
        SECONDMOVESETUP,
        SECONDMOVE,
        THIRDMOVESETUP,
        THIRDMOVE,
        FOURTHMOVESETUP,
        FOURTHMOVE,
        FIFTHMOVESETUP,
        FIFTHMOVE
    }

    @Override
    public void runOpMode() {
        InitialLauncherAndIntakeCode.Launcher launcher = new InitialLauncherAndIntakeCode.Launcher();
        launcher.LaunchMotor = hardwareMap.get(DcMotor.class, "LaunchMotor");
        launcher.LaunchServo = hardwareMap.get(Servo.class, "LaunchServo");
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


        firstAutonomousMode.Chassis autoChassis = new firstAutonomousMode.Chassis();
        firstAutonomousMode.OperState driveOpState = firstAutonomousMode.OperState.FIRSTMOVE;

        double zAngle = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES).firstAngle;
        double yAngle = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES).secondAngle;
        double xAngle = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES).thirdAngle;

        autoChassis.ZeroEncoders();
        autoChassis.Encoders();
        autoChassis.SetAxisMovement();
        double drivePreset = autoChassis.trueDrive + 50;
        double rotationGoal = zAngle;
        

        while (opModeIsActive()) {

            zAngle = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES).firstAngle;
            yAngle = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES).secondAngle;
            xAngle = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES).thirdAngle;

            switch(driveOpState) {
                case FIRSTMOVE:
                    telemetry.addLine("FIRSTMOVE");
                    telemetry.addData("Drive Preset: ", drivePreset);
                    telemetry.addData("drive", autoChassis.trueDrive);
                    autoChassis.Encoders();
                    autoChassis.SetAxisMovement();
                    autoChassis.ForwardAndBackward(drivePreset);


                    if ((Math.abs(zAngle - rotationGoal) >= 2)) {
                        autoChassis.SetMotors(0, 0, autoChassis.CorrectRotation(zAngle, rotationGoal));
                        autoChassis.Drive();
                    }

                    if (Math.abs(drivePreset - autoChassis.trueDrive) <= 0.2) {
                        front_left_wheel.setPower(-0.01);
                        front_right_wheel.setPower(-0.01);
                        back_right_wheel.setPower(-0.01);
                        back_left_wheel.setPower(-0.01);
                        driveOpState = firstAutonomousMode.OperState.SECONDMOVESETUP;
                    }

                    break;

                    
                case SECONDMOVESETUP:
                    telemetry.addLine("SECONDMOVESETUP");
                    while ((Math.abs(zAngle - rotationGoal) >= 2)) {
                        autoChassis.SetMotors(0, 0, autoChassis.CorrectRotation(zAngle, rotationGoal));
                        autoChassis.Drive();
                    }
                    front_left_wheel.setPower(-0.01);
                    front_right_wheel.setPower(-0.01);
                    back_right_wheel.setPower(-0.01);
                    back_left_wheel.setPower(-0.01);
                    autoChassis.Encoders();
                    autoChassis.ZeroEncoders();
                    autoChassis.Encoders();
                    autoChassis.SetAxisMovement();
                    drivePreset = autoChassis.trueDrive - 10;
                    rotationGoal = zAngle;
                    driveOpState = firstAutonomousMode.OperState.SECONDMOVE;

                    break;
                    
                    
                case SECONDMOVE:
                    telemetry.addLine("SECONDMOVE");
                    telemetry.addData("Drive Preset: ", drivePreset);
                    telemetry.addData("drive", autoChassis.trueDrive);
                    autoChassis.Encoders();
                    autoChassis.SetAxisMovement();
                    autoChassis.ForwardAndBackward(drivePreset);



                    if ((Math.abs(zAngle - rotationGoal) >= 2)) {
                        autoChassis.SetMotors(0, 0, autoChassis.CorrectRotation(zAngle, rotationGoal));
                        autoChassis.Drive();
                    }

                    if (Math.abs(drivePreset - autoChassis.trueDrive) <= 0.2) {
                        front_left_wheel.setPower(-0.01);
                        front_right_wheel.setPower(-0.01);
                        back_right_wheel.setPower(-0.01);
                        back_left_wheel.setPower(-0.01);
                        driveOpState = firstAutonomousMode.OperState.THIRDMOVESETUP;
                    }

                    break;


                case THIRDMOVESETUP:
                    telemetry.addLine("THIRDMOVESETUP");
                    while ((Math.abs(zAngle - rotationGoal) >= 2)) {
                        autoChassis.SetMotors(0, 0, autoChassis.CorrectRotation(zAngle, rotationGoal));
                        autoChassis.Drive();
                    }
                    autoChassis.ZeroEncoders();
                    autoChassis.Encoders();
                    autoChassis.SetAxisMovement();
                    drivePreset = autoChassis.trueStrafe - 20;
                    rotationGoal = zAngle;
                    driveOpState = firstAutonomousMode.OperState.THIRDMOVE;
                    break;


                case THIRDMOVE:
                    telemetry.addLine("THIRDMOVE");
                    telemetry.addData("Drive Preset: ", drivePreset);
                    autoChassis.Encoders();
                    autoChassis.SetAxisMovement();
                    autoChassis.LeftAndRight(drivePreset);

                    

                    if ((Math.abs(zAngle - rotationGoal) >= 2)) {
                        autoChassis.SetMotors(0,0,autoChassis.CorrectRotation(zAngle,rotationGoal));
                        autoChassis.Drive();
                    }


                    if (Math.abs(drivePreset - autoChassis.trueStrafe) <= 0.2) {
                        front_left_wheel.setPower(-0.01);
                        front_right_wheel.setPower(-0.01);
                        back_right_wheel.setPower(-0.01);
                        back_left_wheel.setPower(-0.01);
                        driveOpState = firstAutonomousMode.OperState.FOURTHMOVESETUP;
                    }

                    break;

                case FOURTHMOVESETUP:
                    telemetry.addLine("FOURTHMOVESETUP");
                    while ((Math.abs(zAngle - rotationGoal) >= 2)) {
                        autoChassis.SetMotors(0, 0, autoChassis.CorrectRotation(zAngle, rotationGoal));
                        autoChassis.Drive();
                    }
                    front_left_wheel.setPower(-0.01);
                    front_right_wheel.setPower(-0.01);
                    back_right_wheel.setPower(-0.01);
                    back_left_wheel.setPower(-0.01);
                    autoChassis.Encoders();
                    autoChassis.ZeroEncoders();
                    autoChassis.Encoders();
                    autoChassis.SetAxisMovement();
                    drivePreset = autoChassis.trueDrive + 20;
                    rotationGoal = zAngle;
                    driveOpState = OperState.FOURTHMOVE;

                    break;

                case FOURTHMOVE:
                    telemetry.addLine("FOURTHMOVE");
                    telemetry.addData("Drive Preset: ", drivePreset);
                    telemetry.addData("drive", autoChassis.trueDrive);
                    autoChassis.Encoders();
                    autoChassis.SetAxisMovement();
                    autoChassis.ForwardAndBackward(drivePreset);



                    if ((Math.abs(zAngle - rotationGoal) >= 2)) {
                        autoChassis.SetMotors(0, 0, autoChassis.CorrectRotation(zAngle, rotationGoal));
                        autoChassis.Drive();
                    }

                    if (Math.abs(drivePreset - autoChassis.trueDrive) <= 0.2) {
                        front_left_wheel.setPower(-0.01);
                        front_right_wheel.setPower(-0.01);
                        back_right_wheel.setPower(-0.01);
                        back_left_wheel.setPower(-0.01);
                        driveOpState = firstAutonomousMode.OperState.FIFTHMOVESETUP;
                    }

                    break;

                case FIFTHMOVESETUP:
                    telemetry.addLine("FIFTHMOVESETUP");
                    while ((Math.abs(zAngle - rotationGoal) >= 2)) {
                        autoChassis.SetMotors(0, 0, autoChassis.CorrectRotation(zAngle, rotationGoal));
                        autoChassis.Drive();
                    }
                    autoChassis.ZeroEncoders();
                    autoChassis.Encoders();
                    autoChassis.SetAxisMovement();
                    drivePreset = autoChassis.trueStrafe + 30;
                    rotationGoal = zAngle;
                    driveOpState = firstAutonomousMode.OperState.THIRDMOVE;
                    break;

                case FIFTHMOVE:
                    telemetry.addLine("FIFTHMOVE");
                    telemetry.addData("Drive Preset: ", drivePreset);
                    autoChassis.Encoders();
                    autoChassis.SetAxisMovement();
                    autoChassis.LeftAndRight(drivePreset);



                    if ((Math.abs(zAngle - rotationGoal) >= 2)) {
                        autoChassis.SetMotors(0,0,autoChassis.CorrectRotation(zAngle,rotationGoal));
                        autoChassis.Drive();
                    }


                    if (Math.abs(drivePreset - autoChassis.trueStrafe) <= 0.2) {
                        front_left_wheel.setPower(-0.01);
                        front_right_wheel.setPower(-0.01);
                        back_right_wheel.setPower(-0.01);
                        back_left_wheel.setPower(-0.01);
                        System.exit(1);
                    }

                    break;
            }
            telemetry.update();
        }
    }
}
