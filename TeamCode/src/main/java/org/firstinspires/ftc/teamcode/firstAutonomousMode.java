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

    private BNO055IMU imu;

    private DigitalChannel switch_;

    ElapsedTime servoTimer = new ElapsedTime();

    enum OperState {
        FIRSTMOVE,
        SECONDMOVESETUP,
        SECONDMOVE,
        THIRDMOVESETUP,
        THIRDMOVE,
        STARTLAUNCHER,
        FOURTHMOVESETUP,
        FOURTHMOVE,
        SHOOT1,
        Pressed,
        firsttimer,
        Load,
        secondtimer,
        ResetPosition,
        FIFTHMOVESETUP,
        FIFTHMOVE,
        SIXTHMOVESETUP,
        SIXTHMOVE
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
        double drivePreset = 0;
        double rotationGoal = autoChassis.imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES).firstAngle;
        double originalRotation = rotationGoal;
        double shootWait = 0;
        autoChassis.SetRotation(autoChassis.imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES).firstAngle);
        waitForStart();
        servoTimer.reset();

        while (opModeIsActive()) {
            autoChassis.SetRotation(autoChassis.imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES).firstAngle);
            launcher.LauncherRun();

            switch(driveOpState) {
                case FIRSTMOVE:
                    telemetry.addLine("FIRSTMOVE");
                    lift.MoveServo(1);

                    if (servoTimer.time()  >= 5) {
                        lift.MoveServo(0);
                        driveOpState = firstAutonomousMode.OperState.SECONDMOVESETUP;
                    }

                    break;

                    
                case SECONDMOVESETUP:
                    autoChassis.Encoders();
                    autoChassis.ZeroEncoders();
                    autoChassis.Encoders();
                    autoChassis.SetAxisMovement();
                    drivePreset = autoChassis.trueDrive - 60;
                    rotationGoal = autoChassis.zAngle;
                    driveOpState = firstAutonomousMode.OperState.SECONDMOVE;

                    break;

                case SECONDMOVE:
                    telemetry.addLine("SECONDMOVE");
                    telemetry.addData("Drive Preset: ", drivePreset);
                    telemetry.addData("drive", autoChassis.trueDrive);
                    autoChassis.Encoders();
                    autoChassis.SetAxisMovement();
                    autoChassis.ForwardAndBackward(drivePreset);
                    rotationGoal += -0.02;




                    if ((Math.abs(autoChassis.zAngle - rotationGoal) >= 2)) {
                        autoChassis.SetMotors(0, 0, autoChassis.CorrectRotation(autoChassis.zAngle, rotationGoal));
                        autoChassis.Drive();
                    }

                    if (Math.abs(drivePreset - autoChassis.trueDrive) <= 0.2) {
                        autoChassis.front_left_wheel.setPower(-0.01);
                        autoChassis.front_right_wheel.setPower(-0.01);
                        autoChassis.back_right_wheel.setPower(-0.01);
                        autoChassis.back_left_wheel.setPower(-0.01);
                        driveOpState = firstAutonomousMode.OperState.THIRDMOVESETUP;
                    }

                    break;


                case THIRDMOVESETUP:
                    telemetry.addLine("THIRDMOVESETUP");
                    if ((Math.abs(autoChassis.zAngle - rotationGoal) >= 2)) {
                        autoChassis.SetMotors(0, 0, autoChassis.CorrectRotation(autoChassis.zAngle, rotationGoal));
                        autoChassis.Drive();
                        autoChassis.Encoders();
                        autoChassis.ZeroEncoders();
                        autoChassis.SetAxisMovement();
                        rotationGoal = autoChassis.zAngle;
                    }
                    else {
                        autoChassis.front_left_wheel.setPower(-0.01);
                        autoChassis.front_right_wheel.setPower(-0.01);
                        autoChassis.back_right_wheel.setPower(-0.01);
                        autoChassis.back_left_wheel.setPower(-0.01);

                        servoTimer.reset();

                        driveOpState = firstAutonomousMode.OperState.THIRDMOVE;
                    }
                    break;


                case THIRDMOVE:
                    telemetry.addLine("THIRDMOVE");
                    lift.MoveServo(-1);

                    if (servoTimer.time() >= 2) {
                        lift.MoveServo(0);
                        driveOpState = firstAutonomousMode.OperState.STARTLAUNCHER;
                    }

                    break;

                case STARTLAUNCHER:
                    launcher.LauncherToggle();
                    driveOpState = firstAutonomousMode.OperState.FOURTHMOVESETUP;


                case FOURTHMOVESETUP:
                    telemetry.addLine("FIFTHMOVESETUP");
                    telemetry.addData("autoChassis.zAngle", autoChassis.zAngle);
                    telemetry.addData("originalRotaion",originalRotation);
                    telemetry.addData("goal for rotation", originalRotation-180);
                    if ((Math.abs(autoChassis.zAngle - (originalRotation-182)) >= 2)) {
                        autoChassis.SetMotors(0, 0, autoChassis.CorrectRotation(autoChassis.zAngle, (originalRotation-182)));
                        autoChassis.Drive();
                        autoChassis.Encoders();
                        autoChassis.ZeroEncoders();
                        autoChassis.Encoders();
                        autoChassis.SetAxisMovement();
                        rotationGoal = autoChassis.zAngle;
                    }
                    else {
                        autoChassis.front_left_wheel.setPower(-0.01);
                        autoChassis.front_right_wheel.setPower(-0.01);
                        autoChassis.back_right_wheel.setPower(-0.01);
                        autoChassis.back_left_wheel.setPower(-0.01);

                        drivePreset = autoChassis.trueStrafe - 50;


                        driveOpState = firstAutonomousMode.OperState.FOURTHMOVE;
                    }

                    break;

                case FOURTHMOVE:
                    telemetry.addLine("FOURTHMOVE");
                    telemetry.addData("Drive Preset: ", drivePreset);
                    telemetry.addData("strafe", autoChassis.trueStrafe);
                    autoChassis.Encoders();
                    autoChassis.SetAxisMovement();
                    autoChassis.LeftAndRight(drivePreset);





                    if ((Math.abs(autoChassis.zAngle - rotationGoal) >= 2)) {
                        autoChassis.SetMotors(0, 0, autoChassis.CorrectRotation(autoChassis.zAngle, rotationGoal));
                        autoChassis.Drive();
                    }

                    if (Math.abs(drivePreset - autoChassis.trueStrafe) <= 11) {
                        autoChassis.front_left_wheel.setPower(-0.01);
                        autoChassis.front_right_wheel.setPower(-0.01);
                        autoChassis.back_right_wheel.setPower(-0.01);
                        autoChassis.back_left_wheel.setPower(-0.01);
                        servoTimer.reset();
                        driveOpState = firstAutonomousMode.OperState.SHOOT1;
                    }

                    break;

                case SHOOT1:
                    if (shootWait < 6) {
                        if (servoTimer.time() > 0.5) {
                            driveOpState = firstAutonomousMode.OperState.firsttimer;
                        }
                    }
                    else {
                        driveOpState = firstAutonomousMode.OperState.FIFTHMOVESETUP;
                    }



                    break;
                case firsttimer:
                    servoTimer.reset();
                    driveOpState = firstAutonomousMode.OperState.Load;
                    break;

                case Load:
                    launcher.Shoot();
                    if (servoTimer.time() >= 0.15) {
                        driveOpState = firstAutonomousMode.OperState.secondtimer;
                    }
                    break;

                case secondtimer:
                    servoTimer.reset();
                    driveOpState = firstAutonomousMode.OperState.ResetPosition;
                    break;

                case ResetPosition:
                    launcher.Reload();
                    if (servoTimer.time() >= 0.15) {
                        shootWait += 1;
                        servoTimer.reset();
                        driveOpState = firstAutonomousMode.OperState.SHOOT1;
                    }
                    break;

                case FIFTHMOVESETUP:
                    telemetry.addLine("FIFTHMOVESETUP");
                    if ((Math.abs(autoChassis.zAngle - originalRotation) >= 2)) {
                        autoChassis.SetMotors(0, 0, autoChassis.CorrectRotation(autoChassis.zAngle, originalRotation));
                        autoChassis.Drive();
                        autoChassis.Encoders();
                        autoChassis.ZeroEncoders();
                        autoChassis.SetAxisMovement();
                        rotationGoal = autoChassis.zAngle;
                    }
                    else {
                        autoChassis.front_left_wheel.setPower(-0.01);
                        autoChassis.front_right_wheel.setPower(-0.01);
                        autoChassis.back_right_wheel.setPower(-0.01);
                        autoChassis.back_left_wheel.setPower(-0.01);

                        drivePreset = autoChassis.trueDrive + 10;

                        driveOpState = firstAutonomousMode.OperState.FIFTHMOVE;
                    }
                    break;


            }
            telemetry.update();
        }
    }
}
