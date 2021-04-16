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
        LauncherCode.Launcher launcher = new LauncherCode.Launcher();
        LifterCode.Lifter lift = new LifterCode.Lifter();
        ChassisMovementCode.Chassis chassis = new ChassisMovementCode.Chassis();
        DistanceSensorfirstAutonomousMode.OperState driveOpState = DistanceSensorfirstAutonomousMode.OperState.NEWSECONDMOVESETUP;
        DistanceSensorClass.RingClass ring = new DistanceSensorClass.RingClass();

        Control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        expansion_Hub_2 = hardwareMap.get(Blinker.class, "Expansion Hub 2");
        lift.LiftMotor = hardwareMap.get(DcMotor.class, "LiftMotor");

        launcher.LaunchMotor = hardwareMap.get(DcMotor.class, "LaunchMotor");
        launcher.LaunchServo = hardwareMap.get(Servo.class, "LaunchServo");
        chassis.imu = hardwareMap.get(BNO055IMU.class, "imu");
        chassis.front_left_wheel = hardwareMap.get(DcMotor.class, "front left wheel");
        chassis.front_right_wheel = hardwareMap.get(DcMotor.class, "front right wheel");
        chassis.back_left_wheel = hardwareMap.get(DcMotor.class, "back left wheel");
        chassis.back_right_wheel = hardwareMap.get(DcMotor.class, "back right wheel");
        ring.DistanceSensor = hardwareMap.get(DistanceSensor.class, "Distance Sensor");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        chassis.imu.initialize(parameters);
        double drivePreset = 0;
        double strafePreset = 0;
        double rotationGoal = chassis.imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES).firstAngle;
        double originalRotation = rotationGoal;
        double shootWait = 0;
        double rotate = 0;
        double strafe = 0;
        double drive = 0;
        int isRotate = 0;
        int isStrafe = 0;
        int isDrive = 0;
        int ringCount;
        ElapsedTime MultipleUsesTimer = new ElapsedTime();

        double driveValue = 0;
        double strafeValue = 0;
        chassis.SetRotation(chassis.imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES).firstAngle);
        waitForStart();
        servoTimer.reset();

        while (opModeIsActive()) {

            chassis.SetRotation(chassis.imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES).firstAngle);
            launcher.LauncherRun(1);
            ring.MeasureDistance();

            switch(driveOpState) {
                case FIRSTMOVE:
                    telemetry.addLine("FIRSTMOVE");


                    if (servoTimer.time()  >= 5) {

                        driveOpState = DistanceSensorfirstAutonomousMode.OperState.SECONDMOVESETUP;
                    }
                    break;

               /* case NEWSECONDMOVESETUP:
                    chassis.SetAxisMovement();
                    chassis.ZeroEncoders();
                    chassis.SetAxisMovement();
                    chassis.SetPresetMovement(60, 1,50, 1,chassis.zAngle);
                    servoTimer.reset();
                    chassis.movementTimer.reset();
                    driveOpState = DistanceSensorfirstAutonomousMode.OperState.NEWSECONDMOVE;
                    break;
*/
                case NEWSECONDMOVE:
                    telemetry.addLine("newsecondmoveE");
                    telemetry.addData("driveStrafe", drivePreset);
                    telemetry.addData("strafePreset", strafePreset);
                    telemetry.addData("rotate", chassis.rotation);
                    telemetry.addData("drivevalue", chassis.trueDrive);
                    telemetry.addData("strafevalue", chassis.trueStrafe);
                    telemetry.addData("drive", chassis.drive);
                    telemetry.addData("strafe", chassis.strafe);
                    telemetry.addData("front left wheel", chassis.frontLeft + " = "+(1*chassis.drive) +" + "+(1*chassis.strafe)+" + "+(-1*chassis.rotation));
                    telemetry.addData("back left wheel", chassis.backLeft + " = "+(-1*chassis.drive) +" + "+(1*chassis.strafe)+" + "+(1*chassis.rotation));
                    telemetry.addData("front right wheel", chassis.frontRight + " = "+(1*chassis.drive) +" + "+(-1*chassis.strafe)+" + "+(1*chassis.rotation));
                    telemetry.addData("back right wheel", chassis.backRight + " = "+(1*chassis.drive) +" + "+(1*chassis.strafe)+" + "+(1*chassis.rotation));
                    telemetry.addData("firstSignumRotate", Math.signum(rotationGoal - chassis.zAngle));
                    telemetry.addData("firstSignumStrafe", Math.signum(strafePreset - chassis.trueStrafe));
                    telemetry.addData("firstSignumDrive", Math.signum(drivePreset - chassis.trueDrive));
                    telemetry.addData("Math.maxRotate", (Math.max(0.2, Math.abs((rotationGoal - chassis.zAngle) / 180))));
                    telemetry.addData("Math.maxStrafe", (Math.max(0.2, Math.abs((strafePreset - chassis.trueStrafe) / strafePreset))));
                    telemetry.addData("Math.maxDrive", (Math.max(0.2, Math.abs((drivePreset - chassis.trueDrive) / drivePreset))));

                    if (chassis.MoveToLocation() == true) {
                        telemetry.addLine("done");
                    }

                    break;
                /*
                case SECONDMOVESETUP:
                    chassis.Encoders();
                    chassis.ZeroEncoders();
                    chassis.Encoders();
                    chassis.SetAxisMovement();
                    drivePreset = chassis.trueDrive - 60;
                    rotationGoal = chassis.zAngle;
                    driveOpState = DistanceSensorfirstAutonomousMode.OperState.SECONDMOVE;
                    break;

                case SECONDMOVE:
                    telemetry.addLine("SECONDMOVE");
                    telemetry.addData("Drive Preset: ", drivePreset);
                    telemetry.addData("drive", chassis.trueDrive);
                    chassis.Encoders();
                    chassis.SetAxisMovement();
                    chassis.ForwardAndBackward(drivePreset);
                    rotationGoal += -0.02;

                    if ((Math.abs(chassis.zAngle - rotationGoal) >= 2)) {
                        chassis.SetMotors(0, 0, chassis.CorrectRotation(chassis.zAngle, rotationGoal));
                        chassis.Drive();
                    }

                    if (Math.abs(drivePreset - chassis.trueDrive) <= 0.2) {
                        chassis.front_left_wheel.setPower(-0.01);
                        chassis.front_right_wheel.setPower(-0.01);
                        chassis.back_right_wheel.setPower(-0.01);
                        chassis.back_left_wheel.setPower(-0.01);
                        driveOpState = DistanceSensorfirstAutonomousMode.OperState.THIRDMOVESETUP;
                    }
                    break;


                case RESETTIMER:
                    MultipleUsesTimer.reset();
                    driveOpState = DistanceSensorfirstAutonomousMode.OperState.MEASURE;
                    break;

                case MEASURE:
                    if (MultipleUsesTimer.time(TimeUnit.SECONDS) >= 1) {
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
                    if ((Math.abs(chassis.zAngle - rotationGoal) >= 2)) {
                        chassis.SetMotors(0, 0, chassis.CorrectRotation(chassis.zAngle, rotationGoal));
                        chassis.Drive();
                        chassis.Encoders();
                        chassis.ZeroEncoders();
                        chassis.SetAxisMovement();
                        rotationGoal = chassis.zAngle;
                    }
                    else {
                        chassis.front_left_wheel.setPower(-0.01);
                        chassis.front_right_wheel.setPower(-0.01);
                        chassis.back_right_wheel.setPower(-0.01);
                        chassis.back_left_wheel.setPower(-0.01);

                        servoTimer.reset();

                        driveOpState = DistanceSensorfirstAutonomousMode.OperState.THIRDMOVE;
                    }
                    break;

                case THIRDMOVE:
                    telemetry.addLine("THIRDMOVE");
                    ;

                    if (servoTimer.time() >= 2) {

                        driveOpState = DistanceSensorfirstAutonomousMode.OperState.STARTLAUNCHER;
                    }
                    break;

                case STARTLAUNCHER:
                    launcher.LauncherToggle();
                    driveOpState = DistanceSensorfirstAutonomousMode.OperState.FOURTHMOVESETUP;

                case FOURTHMOVESETUP:
                    telemetry.addLine("FIFTHMOVESETUP");
                    telemetry.addData("chassis.zAngle", chassis.zAngle);
                    telemetry.addData("originalRotation",originalRotation);
                    telemetry.addData("goal for rotation", originalRotation-180);
                    if ((Math.abs(chassis.zAngle - (originalRotation-182)) >= 2)) {
                        chassis.SetMotors(0, 0, chassis.CorrectRotation(chassis.zAngle, (originalRotation-182)));
                        chassis.Drive();
                        chassis.Encoders();
                        chassis.ZeroEncoders();
                        chassis.Encoders();
                        chassis.SetAxisMovement();
                        rotationGoal = chassis.zAngle;
                    }
                    else {
                        chassis.front_left_wheel.setPower(-0.01);
                        chassis.front_right_wheel.setPower(-0.01);
                        chassis.back_right_wheel.setPower(-0.01);
                        chassis.back_left_wheel.setPower(-0.01);

                        drivePreset = chassis.trueStrafe - 50;

                        driveOpState = DistanceSensorfirstAutonomousMode.OperState.FOURTHMOVE;
                    }
                    break;

                case FOURTHMOVE:
                    telemetry.addLine("FOURTHMOVE");
                    telemetry.addData("Drive Preset: ", drivePreset);
                    telemetry.addData("strafe", chassis.trueStrafe);
                    chassis.Encoders();
                    chassis.SetAxisMovement();
                    chassis.LeftAndRight(drivePreset);

                    if ((Math.abs(chassis.zAngle - rotationGoal) >= 2)) {
                        chassis.SetMotors(0, 0, chassis.CorrectRotation(chassis.zAngle, rotationGoal));
                        chassis.Drive();
                    }

                    if (Math.abs(drivePreset - chassis.trueStrafe) <= 11) {
                        chassis.front_left_wheel.setPower(-0.01);
                        chassis.front_right_wheel.setPower(-0.01);
                        chassis.back_right_wheel.setPower(-0.01);
                        chassis.back_left_wheel.setPower(-0.01);
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
                    if ((Math.abs(chassis.zAngle - originalRotation) >= 2)) {
                        chassis.SetMotors(0, 0, chassis.CorrectRotation(chassis.zAngle, originalRotation));
                        chassis.Drive();
                        chassis.Encoders();
                        chassis.ZeroEncoders();
                        chassis.SetAxisMovement();
                        rotationGoal = chassis.zAngle;
                    }
                    else {
                        chassis.front_left_wheel.setPower(-0.01);
                        chassis.front_right_wheel.setPower(-0.01);
                        chassis.back_right_wheel.setPower(-0.01);
                        chassis.back_left_wheel.setPower(-0.01);

                        drivePreset = chassis.trueDrive + 10;

                        driveOpState = DistanceSensorfirstAutonomousMode.OperState.FIFTHMOVE;
                    }
                    break;

                 */
            }
            telemetry.update();
        }
    }
}
