package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp

public class ChassisMovementCodeIncludingLifter extends LinearOpMode {

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
        double trueStrafe;
        double slowIntensity = 10;
        double trueRotate;
        double backRightMultiplier = 1;
        double backLeftMultiplier = 1;
        double frontRightMultiplier = 0.8;
        double frontLeftMultiplier = 0.8;

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
        NORMALDRIVE,
        NORMALROTATE,
        FORWARD,
        LATERALMOVEMENT,
        SETMOVEMENTDISTANCE
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
        double movementLength = 0;
        double forwardLength;
        double lateralMovement;
        double increaseIntensity = 5;
        boolean upWait = false;
        boolean downWait = false;
        boolean rightWait = false;
        boolean leftWait = false;
        double drivePreset = 0;
        ChassisMovementCodeIncludingLifter.Chassis chasty = new ChassisMovementCodeIncludingLifter.Chassis();
        ChassisMovementCodeIncludingLifter.OperState driveOpState = ChassisMovementCodeIncludingLifter.OperState.NORMALDRIVE;
        InitialLifterCode.
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
                        driveOpState = ChassisMovementCodeIncludingLifter.OperState.NORMALROTATE;
                    }

                    if (this.gamepad1.a) {
                        drivePreset = chasty.trueDrive + movementLength;
                        driveOpState = ChassisMovementCodeIncludingLifter.OperState.FORWARD;
                    }

                    if (this.gamepad1.b) {
                        driveOpState = ChassisMovementCodeIncludingLifter.OperState.LATERALMOVEMENT;
                        drivePreset = chasty.trueStrafe + movementLength;
                    }

                    if (this.gamepad1.y) {
                        driveOpState = ChassisMovementCodeIncludingLifter.OperState.SETMOVEMENTDISTANCE;
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
                        driveOpState = ChassisMovementCodeIncludingLifter.OperState.NORMALDRIVE;
                    }
                    break;

                case FORWARD:

                    chasty.SetAxisMovement();
                    chasty.Encoders();
                    chasty.ForwardAndBackward(drivePreset);

                    if (this.gamepad1.right_trigger != 0) {
                        driveOpState = ChassisMovementCodeIncludingLifter.OperState.NORMALDRIVE;
                    }

                    if (Math.abs(drivePreset - chasty.trueDrive) <= 0.2) {
                        front_left_wheel.setPower(0.01);
                        front_right_wheel.setPower(0.01);
                        back_right_wheel.setPower(0.01);
                        back_left_wheel.setPower(0.01);
                        driveOpState = ChassisMovementCodeIncludingLifter.OperState.NORMALDRIVE;
                    }


                    break;

                case LATERALMOVEMENT:

                    chasty.SetAxisMovement();
                    chasty.Encoders();
                    chasty.LeftAndRight(drivePreset);

                    if (this.gamepad1.right_trigger != 0) {
                        driveOpState = ChassisMovementCodeIncludingLifter.OperState.NORMALDRIVE;
                    }

                    if (Math.abs(drivePreset - chasty.trueStrafe) <= 0.2) {
                        front_left_wheel.setPower(0.01);
                        front_right_wheel.setPower(0.01);
                        back_right_wheel.setPower(0.01);
                        back_left_wheel.setPower(0.01);
                        driveOpState = ChassisMovementCodeIncludingLifter.OperState.NORMALDRIVE;
                    }

                    break;
                case SETMOVEMENTDISTANCE:
                    telemetry.addLine("Press up and down on the d-pad to increase movement per press");
                    telemetry.addLine("Press right and left on the d-pad to increase or decrease the increased amount added");
                    telemetry.addData("Current movement per press", movementLength);
                    telemetry.addData("Amount increased per increase", increaseIntensity);
                    telemetry.update();
                    if ((upWait) & (!this.gamepad1.dpad_up)) {
                        movementLength = movementLength + increaseIntensity;
                        upWait = false;
                    }
                    if ((!this.gamepad1.dpad_down) & (downWait)) {
                        movementLength = movementLength - increaseIntensity;
                        downWait = false;
                    }
                    if ((!this.gamepad1.dpad_right) & (rightWait)) {
                        increaseIntensity = increaseIntensity + 1;
                        rightWait = false;
                    }
                    if ((!this.gamepad1.dpad_left) & (leftWait)) {
                        increaseIntensity = increaseIntensity - 1;
                        leftWait = false;
                    }

                    if (this.gamepad1.dpad_up) {
                        upWait = true;
                    }
                    if (this.gamepad1.dpad_down) {
                        downWait = true;
                    }
                    if (this.gamepad1.dpad_right) {
                        rightWait =true;
                    }
                    if (this.gamepad1.dpad_left) {
                        leftWait = true;
                    }

                    if (this.gamepad1.x) {
                        driveOpState = ChassisMovementCodeIncludingLifter.OperState.NORMALDRIVE;
                    }

                    break;

                default :
                    break;
            }
        }
    }
}
