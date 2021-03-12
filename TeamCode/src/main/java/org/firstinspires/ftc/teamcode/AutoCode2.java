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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.concurrent.TimeUnit;

//Created by mostly Patrick, partly Ethan
@Autonomous
public class AutoCode2 extends LinearOpMode {

    private Blinker Control_Hub;
    private Blinker expansion_Hub_2;
    ElapsedTime servoTimer = new ElapsedTime();

    enum OperState {
        FIRSTMOVE,
        PREPMOVEANDLIFT,
        LIFTUP,
        DECIDE,
        LIFTDOWN,
        PREPMOVERIGHT,
        PREPMOVEBACK,
        MOVEANDLIFT,
        MOVERIGHT,
        MOVEBACK,
        NEWSECONDMOVESETUP,
        SECONDMOVESETUP,
        THIRDMOVESETUP,
        THIRDMOVE,
        STARTLAUNCHER,
        FOURTHMOVESETUP,
        FOURTHMOVE,
        SHOOT1,
        RESETTIMER,
        MEASURE,
        firsttimer,
        Load,
        secondtimer,
        ResetPosition,
        FIFTHMOVESETUP,
        PREPA,
        PREPB,
        PREPC,
        A,
        B,
        C
    }

    @Override
    public void runOpMode() {
        InitialLauncherAndIntakeCode.Launcher launcher = new InitialLauncherAndIntakeCode.Launcher();
        InitialLifterCode.Lifter lift = new InitialLifterCode.Lifter();
        ChassisMovementCode.Chassis autoChassis = new ChassisMovementCode.Chassis();
        AutoCode2.OperState driveOpState = AutoCode2.OperState.FIRSTMOVE;
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
        int ringCount = 0;
        ElapsedTime MeasureWait = new ElapsedTime();
        double driveValue = 0;
        double strafeValue = 0;
        autoChassis.SetRotation(autoChassis.imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES).firstAngle);
        waitForStart();
        servoTimer.reset();
/*                    if (autoChassis.MoveToLocation() == true) {
                        telemetry.addLine("done");
                    }
*/
        while (opModeIsActive()) {

            autoChassis.SetRotation(autoChassis.imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES).firstAngle);
            launcher.LauncherRun();
            ring.MeasureDistance();

            switch(driveOpState) {
                case FIRSTMOVE:
                    telemetry.addLine("FIRSTMOVE");
                    lift.MoveServo(1);

                    if (servoTimer.time()  >= 2) {
                        lift.MoveServo(0);
                        driveOpState = AutoCode2.OperState.PREPMOVEANDLIFT;
                    }
                    break;

                case RESETTIMER:
                    MeasureWait.reset();
                    driveOpState = AutoCode2.OperState.MEASURE;
                    break;

                case PREPMOVEANDLIFT:
                    autoChassis.SetAxisMovement();
                    autoChassis.ZeroEncoders();
                    autoChassis.SetAxisMovement();
                    autoChassis.SetPresetMovement(-19,0, autoChassis.zAngle);
                    servoTimer.reset();
                    driveOpState = OperState.MOVEANDLIFT;
                    break;

                case MOVEANDLIFT:
                    telemetry.addLine("newsecondmoveE");
                    telemetry.addData("driveStrafe", drivePreset);
                    telemetry.addData("strafePreset", strafePreset);
                    telemetry.addData("rotate", autoChassis.rotation);
                    telemetry.addData("drivevalue", autoChassis.trueDrive);
                    telemetry.addData("strafevalue", autoChassis.trueStrafe);
                    telemetry.addData("drive", autoChassis.drive);
                    telemetry.addData("strafe", autoChassis.strafe);
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
                        telemetry.addLine("done");
                        servoTimer.reset();
                        driveOpState = OperState.LIFTUP;
                    }

                    break;
                case LIFTUP:
                    lift.MoveServo(-1);
                    if (servoTimer.time()  >= 2) {
                        lift.MoveServo(0);
                        driveOpState = AutoCode2.OperState.PREPMOVERIGHT;
                    }
                    break;
                case PREPMOVERIGHT:
                    autoChassis.SetAxisMovement();
                    autoChassis.ZeroEncoders();
                    autoChassis.SetAxisMovement();
                    autoChassis.SetPresetMovement(0,17, autoChassis.zAngle);
                    servoTimer.reset();
                    driveOpState = OperState.MOVERIGHT;
                    break;
                case MOVERIGHT:
                    if (autoChassis.MoveToLocation() == true) {
                        driveOpState = OperState.MEASURE;
                    }
                    break;
                case MEASURE:
                    MeasureWait.reset();
                    if (MeasureWait.time(TimeUnit.SECONDS) >= 1) {
                      ring.MeasureDistance();
                      ringCount = ring.RingHeight();
                      driveOpState = OperState.PREPMOVEBACK;
                    }
                    break;
                case PREPMOVEBACK:
                    autoChassis.SetAxisMovement();
                    autoChassis.ZeroEncoders();
                    autoChassis.SetAxisMovement();
                    autoChassis.SetPresetMovement(0,-17, autoChassis.zAngle);
                    driveOpState = OperState.MOVEBACK;
                    break;
                case MOVEBACK:
                    if (autoChassis.MoveToLocation() == true) {
                        servoTimer.reset();
                        driveOpState = OperState.LIFTDOWN;
                    }
                    break;
                case LIFTDOWN:
                    lift.MoveServo(1);
                    if (servoTimer.time()  >= 2) {
                        lift.MoveServo(0);
                        driveOpState = AutoCode2.OperState.DECIDE;
                    }
                    break;
                case DECIDE:
                    if (ringCount == 0) {
                        driveOpState = AutoCode2.OperState.PREPA;
                    }
                    else if (ringCount == 1) {
                        driveOpState = AutoCode2.OperState.PREPB;
                    }
                    else if (ringCount == 4) {
                        driveOpState = AutoCode2.OperState.PREPC;
                    }
                    break;
                case PREPA:
                    autoChassis.SetAxisMovement();
                    autoChassis.ZeroEncoders();
                    autoChassis.SetAxisMovement();
                    autoChassis.SetPresetMovement(-38.5,-7, autoChassis.zAngle);
                    driveOpState = OperState.A;
                    break;
                case PREPB:
                    autoChassis.SetAxisMovement();
                    autoChassis.ZeroEncoders();
                    autoChassis.SetAxisMovement();
                    autoChassis.SetPresetMovement(-61,9.5, autoChassis.zAngle);
                    driveOpState = OperState.B;
                    break;
                case PREPC:
                    autoChassis.SetAxisMovement();
                    autoChassis.ZeroEncoders();
                    autoChassis.SetAxisMovement();
                    autoChassis.SetPresetMovement(-82,-7, autoChassis.zAngle);
                    driveOpState = OperState.C;
                    break;
                case A:
                    if (autoChassis.MoveToLocation() == true) { }
                    break;
                case B:
                    if (autoChassis.MoveToLocation() == true) { }
                    break;
                case C:
                    if (autoChassis.MoveToLocation() == true) { }
                    break;
                case THIRDMOVE:
                    telemetry.addLine("THIRDMOVE");
                    lift.MoveServo(-1);

                    if (servoTimer.time() >= 2) {
                        lift.MoveServo(0);
                        driveOpState = AutoCode2.OperState.STARTLAUNCHER;
                    }
                    break;

                case STARTLAUNCHER:
                    launcher.LauncherToggle();
                    driveOpState = AutoCode2.OperState.FOURTHMOVESETUP;

                case SHOOT1:
                    if (shootWait < 6) {
                        if (servoTimer.time() > 0.5) {
                            driveOpState = AutoCode2.OperState.firsttimer;
                        }
                    }
                    else {
                        driveOpState = AutoCode2.OperState.FIFTHMOVESETUP;
                    }

                    break;
                case firsttimer:
                    servoTimer.reset();
                    driveOpState = AutoCode2.OperState.Load;
                    break;

                case Load:
                    launcher.Shoot();
                    if (servoTimer.time() >= 0.15) {
                        driveOpState = AutoCode2.OperState.secondtimer;
                    }
                    break;

                case secondtimer:
                    servoTimer.reset();
                    driveOpState = AutoCode2.OperState.ResetPosition;
                    break;

                case ResetPosition:
                    launcher.Reload();
                    if (servoTimer.time() >= 0.15) {
                        shootWait += 1;
                        servoTimer.reset();
                        driveOpState = AutoCode2.OperState.SHOOT1;
                    }
                    break;

            }
            telemetry.update();
        }
    }
}
