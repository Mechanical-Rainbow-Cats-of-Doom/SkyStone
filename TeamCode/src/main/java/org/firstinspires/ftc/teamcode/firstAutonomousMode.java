package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;  //This is the package for controlling the IMU
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.lang.Math;  //This is the standard Java package for a variety of math functions
import java.lang.annotation.ElementType;

@Autonomous
public class firstAutonomousMode extends LinearOpMode {

    private Blinker Control_Hub;
    private Blinker expansion_Hub_2;

    //motors
    private DcMotor back_right_wheel;
    private DcMotor front_right_wheel;
    private DcMotor back_left_wheel;
    private DcMotor front_left_wheel;

    private BNO055IMU imu;

    private DigitalChannel switch_;

    ElapsedTime servoTimer = new ElapsedTime();

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
        InitialLifterCode.Lifter lift = new InitialLifterCode.Lifter();
        ChassisMovementCode.Chassis autoChassis = new ChassisMovementCode.Chassis();
        firstAutonomousMode.OperState driveOpState = firstAutonomousMode.OperState.FIRSTMOVE;


        Control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        expansion_Hub_2 = hardwareMap.get(Blinker.class, "Expansion Hub 2");
        lift.LiftMotor = hardwareMap.get(DcMotor.class, "LiftMotor");
        lift.ForkServo = hardwareMap.get(CRServo.class, "LiftServo");
        launcher.LaunchMotor = hardwareMap.get(DcMotor.class, "LaunchMotor");
        launcher.LaunchServo = hardwareMap.get(Servo.class, "LaunchServo");
        autoChassis.imu = hardwareMap.get(BNO055IMU.class, "imu");
        autoChassis.front_left_wheel = hardwareMap.get(DcMotor.class, "front left wheel");
        autoChassis.front_right_wheel = hardwareMap.get(DcMotor.class, "front right wheel");
        autoChassis.back_left_wheel = hardwareMap.get(DcMotor.class, "back left wheel");
        autoChassis.back_right_wheel = hardwareMap.get(DcMotor.class, "back right wheel");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();  //in wrong spot--where is better?
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        autoChassis.imu.initialize(parameters);

        waitForStart();



        double zAngle = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES).firstAngle;
        double yAngle = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES).secondAngle;
        double xAngle = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES).thirdAngle;

        servoTimer.reset();
        double drivePreset = 0;
        double rotationGoal = autoChassis.imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES).firstAngle;
        

        while (opModeIsActive()) {

            zAngle = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES).firstAngle;
            yAngle = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES).secondAngle;
            xAngle = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES).thirdAngle;

            switch(driveOpState) {
                case FIRSTMOVE:
                    telemetry.addLine("FIRSTMOVE");
                    lift.MoveServo(1);

                    if (servoTimer.seconds() == 5.9) {
                        driveOpState = firstAutonomousMode.OperState.SECONDMOVESETUP;
                    }

                    break;

                    
                case SECONDMOVESETUP:
                    autoChassis.Encoders();
                    autoChassis.ZeroEncoders();
                    autoChassis.Encoders();
                    autoChassis.SetAxisMovement();
                    drivePreset = autoChassis.trueDrive - 50;
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
                    drivePreset = autoChassis.trueStrafe + 10;
                    rotationGoal = zAngle;
                    driveOpState = firstAutonomousMode.OperState.THIRDMOVE;
                    break;


                case THIRDMOVE:
                    telemetry.addLine("THIRDMOVE");
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
                    servoTimer.reset();
                    rotationGoal = zAngle;
                    driveOpState = firstAutonomousMode.OperState.SECONDMOVE;

                    break;

                case FOURTHMOVE:
                    telemetry.addLine("FOURTHMOVE");
                    lift.MoveServo(-1);

                    if (servoTimer.seconds() == 5.7) {
                        System.exit(1);
                    }

                    break;
            }
            telemetry.update();
        }
    }
}
