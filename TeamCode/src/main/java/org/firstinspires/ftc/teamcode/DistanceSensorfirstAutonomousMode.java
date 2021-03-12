package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.lang.Math;
import java.util.concurrent.TimeUnit;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

//Created by mostly Patrick, partly Ethan
@Autonomous
public class DistanceSensorfirstAutonomousMode extends LinearOpMode {

    private Blinker Control_Hub;
    private Blinker expansion_Hub_2;
    ElapsedTime servoTimer = new ElapsedTime();

    enum OperState {
        FIRSTMOVE,
        NEWSECONDMOVESETUP,
        NEWSECONDMOVE,
        SECONDMOVESETUP,
        SECONDMOVE,
        THIRDMOVESETUP,
        THIRDMOVE,
        STARTLAUNCHER,
        FOURTHMOVESETUP,
        FOURTHMOVE,
        SHOOT1,
        RESETTIMER,
        MEASURE,
        BOTTOM0,
        MIDDLE1,
        TOP4,
        Pressed,
        firsttimer,
        Load,
        secondtimer,
        ResetPosition,
        FIFTHMOVESETUP,
        FIFTHMOVE,
        A,
        B,
        C,
        SIXTHMOVESETUP,
        SIXTHMOVE
    }

    @Override
    public void runOpMode() {
        InitialLauncherAndIntakeCode.Launcher launcher = new InitialLauncherAndIntakeCode.Launcher();
        InitialLifterCode.Lifter lift = new InitialLifterCode.Lifter();
        ChassisMovementCode.Chassis autoChassis = new ChassisMovementCode.Chassis();
        DistanceSensorfirstAutonomousMode.OperState driveOpState = DistanceSensorfirstAutonomousMode.OperState.NEWSECONDMOVESETUP;
        DistanceSensorClass.RingClass ring = new DistanceSensorClass.RingClass();

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
        ring.DistanceSensor = hardwareMap.get(DistanceSensor.class, "Distance Sensor");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        autoChassis.imu.initialize(parameters);
        double drivePreset = 0;
        double strafePreset = 0;
        double rotationGoal = autoChassis.imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES).firstAngle;
        double originalRotation = rotationGoal;
        double shootWait = 0;
        double rotate = 0;
        double strafe = 0;
        double drive = 0;
        int isRotate = 0;
        int isStrafe = 0;
        int isDrive = 0;
        int ringCount;
        ElapsedTime MeasureWait = new ElapsedTime();

        double driveValue = 0;
        double strafeValue = 0;
        autoChassis.SetRotation(autoChassis.imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES).firstAngle);
        waitForStart();
        servoTimer.reset();

        while (opModeIsActive()) {

            autoChassis.SetRotation(autoChassis.imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES).firstAngle);
            launcher.LauncherRun();
            ring.MeasureDistance();

            switch(driveOpState) {
                case FIRSTMOVE:
                    telemetry.addLine("FIRSTMOVE");
                    lift.MoveServo(1);

                    if (servoTimer.time()  >= 5) {
                        lift.MoveServo(0);
                        driveOpState = DistanceSensorfirstAutonomousMode.OperState.SECONDMOVESETUP;
                    }
                    break;

                case NEWSECONDMOVESETUP:
                    autoChassis.SetAxisMovement();
                    autoChassis.ZeroEncoders();
                    autoChassis.SetAxisMovement();
                    autoChassis.SetPresetMovement(-80,-50,autoChassis.zAngle);
                    servoTimer.reset();
                    driveOpState = DistanceSensorfirstAutonomousMode.OperState.NEWSECONDMOVE;
                    break;

                case NEWSECONDMOVE:
                    telemetry.addLine("newsecondmoveE");
                    telemetry.addData("driveStrafe", drivePreset);
                    telemetry.addData("strafePreset", strafePreset);
                    telemetry.addData("rotate", rotate);
                    telemetry.addData("drivevalue", autoChassis.trueDrive);
                    telemetry.addData("strafevalue", autoChassis.trueStrafe);
                    telemetry.addData("drive", drive);
                    telemetry.addData("strafe", strafe);
                    telemetry.addData("back left wheel", autoChassis.backLeft);
                    telemetry.addData("back right wheel", autoChassis.backRight);
                    telemetry.addData("front right wheel", autoChassis.frontRight);
                    telemetry.addData("front left wheel", autoChassis.frontLeft);
                    telemetry.addData("firstSignumRotate", Math.signum(rotationGoal - autoChassis.zAngle));
                    telemetry.addData("firstSignumStrafe", Math.signum(strafePreset - autoChassis.trueStrafe));
                    telemetry.addData("firstSignumDrive", Math.signum(drivePreset - autoChassis.trueDrive));
                    telemetry.addData("Math.maxRotate", (Math.max(0.2, Math.abs((rotationGoal - autoChassis.zAngle) / 180))));
                    telemetry.addData("Math.maxStrafe", (Math.max(0.2, Math.abs((strafePreset - autoChassis.trueStrafe) / strafePreset))));
                    telemetry.addData("Math.maxDrive", (Math.max(0.2, Math.abs((drivePreset - autoChassis.trueDrive) / drivePreset))));

                    if (autoChassis.MoveToLocation() == true) {

                    }

                    break;
                /*
                case SECONDMOVESETUP:
                    autoChassis.Encoders();
                    autoChassis.ZeroEncoders();
                    autoChassis.Encoders();
                    autoChassis.SetAxisMovement();
                    drivePreset = autoChassis.trueDrive - 60;
                    rotationGoal = autoChassis.zAngle;
                    driveOpState = DistanceSensorfirstAutonomousMode.OperState.SECONDMOVE;
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
                        driveOpState = DistanceSensorfirstAutonomousMode.OperState.THIRDMOVESETUP;
                    }
                    break;


                case RESETTIMER:
                    MeasureWait.reset();
                    driveOpState = DistanceSensorfirstAutonomousMode.OperState.MEASURE;
                    break;

                case MEASURE:
                    if (MeasureWait.time(TimeUnit.SECONDS) >= 1) {
                      ring.MeasureDistance();
                      ringCount = ring.RingHeight();
                      if (ringCount == 0) {
                          driveOpState = DistanceSensorfirstAutonomousMode.OperState.A;
                      }
                      else if (ringCount == 1) {
                          driveOpState = DistanceSensorfirstAutonomousMode.OperState.B;
                      }
                      else if (ringCount == 4) {
                          driveOpState = DistanceSensorfirstAutonomousMode.OperState.C;
                      }
                    }
                    break;
                case A:
                    //Moving to A Code
                    break;
                case B:
                    //Moving to B Code
                    break;
                case C:
                    //Moving to C Code
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

                        driveOpState = DistanceSensorfirstAutonomousMode.OperState.THIRDMOVE;
                    }
                    break;

                case THIRDMOVE:
                    telemetry.addLine("THIRDMOVE");
                    lift.MoveServo(-1);

                    if (servoTimer.time() >= 2) {
                        lift.MoveServo(0);
                        driveOpState = DistanceSensorfirstAutonomousMode.OperState.STARTLAUNCHER;
                    }
                    break;

                case STARTLAUNCHER:
                    launcher.LauncherToggle();
                    driveOpState = DistanceSensorfirstAutonomousMode.OperState.FOURTHMOVESETUP;

                case FOURTHMOVESETUP:
                    telemetry.addLine("FIFTHMOVESETUP");
                    telemetry.addData("autoChassis.zAngle", autoChassis.zAngle);
                    telemetry.addData("originalRotation",originalRotation);
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

                        driveOpState = DistanceSensorfirstAutonomousMode.OperState.FOURTHMOVE;
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
                        driveOpState = DistanceSensorfirstAutonomousMode.OperState.SHOOT1;
                    }
                    break;

                case SHOOT1:
                    if (shootWait < 6) {
                        if (servoTimer.time() > 0.5) {
                            driveOpState = DistanceSensorfirstAutonomousMode.OperState.firsttimer;
                        }
                    }
                    else {
                        driveOpState = DistanceSensorfirstAutonomousMode.OperState.FIFTHMOVESETUP;
                    }

                    break;
                case firsttimer:
                    servoTimer.reset();
                    driveOpState = DistanceSensorfirstAutonomousMode.OperState.Load;
                    break;

                case Load:
                    launcher.Shoot();
                    if (servoTimer.time() >= 0.15) {
                        driveOpState = DistanceSensorfirstAutonomousMode.OperState.secondtimer;
                    }
                    break;

                case secondtimer:
                    servoTimer.reset();
                    driveOpState = DistanceSensorfirstAutonomousMode.OperState.ResetPosition;
                    break;

                case ResetPosition:
                    launcher.Reload();
                    if (servoTimer.time() >= 0.15) {
                        shootWait += 1;
                        servoTimer.reset();
                        driveOpState = DistanceSensorfirstAutonomousMode.OperState.SHOOT1;
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

                        driveOpState = DistanceSensorfirstAutonomousMode.OperState.FIFTHMOVE;
                    }
                    break;

                 */
            }
            telemetry.update();
        }
    }
}
