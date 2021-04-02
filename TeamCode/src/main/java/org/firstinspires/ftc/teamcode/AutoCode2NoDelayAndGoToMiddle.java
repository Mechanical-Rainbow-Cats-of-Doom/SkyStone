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

//Created by mostly Ethan, partly Patrick
@Autonomous
public class AutoCode2NoDelayAndGoToMiddle extends LinearOpMode {

    private Blinker Control_Hub;
    private Blinker expansion_Hub_2;
    ElapsedTime servoTimer = new ElapsedTime();
    ElapsedTime menuTimer = new ElapsedTime();

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
        GoToTargetZone,
        PrepMoveToPowerShots,
        PrepStrafeLeft,
        PrepLaunchPark,
        PrepMoveToGoals,
        MoveToPowerShots,
        StrafeLeft,
        LaunchPark,
        MoveToGoals,
        NextLocation,
        PrepSpinAround,
        SpinAround,
        Launch,
        Delayer,
        Reload
    }

    enum Menu {
        StartLocation,
        DelayAndGo,
        ButtonWaiter0,
        ButtonWaiter1,
        Powershots,
        ButtonWaiter2,
        OnlyPark,
        ButtonWaiter3,
        Goals,
        ButtonWaiter4,
        AreYouMoving,
        ButtonWaiter5,
        CheckForInvalid,
        Save,
        AskIfDone,
        Redo,
        ButtonWaiter6
    }

    @Override
    public void runOpMode() {
        InitialLauncherAndIntakeCode.Launcher launcher = new InitialLauncherAndIntakeCode.Launcher();
        InitialLifterCode.Lifter lift = new InitialLifterCode.Lifter();
        ChassisMovementCode.Chassis autoChassis = new ChassisMovementCode.Chassis();
        AutoCode2NoDelayAndGoToMiddle.OperState driveOpState = AutoCode2NoDelayAndGoToMiddle.OperState.FIRSTMOVE;
        AutoCode2NoDelayAndGoToMiddle.Menu menu = AutoCode2NoDelayAndGoToMiddle.Menu.StartLocation;
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
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        autoChassis.imu.initialize(parameters);
        double drivePreset = 0;

        //menu variables
        boolean IsMenuDone = false;
        boolean OnRed = true; //true is on red, false is on blue.
        boolean DoneMeasuring = false;
        int StartLocation = 0; //1 is Blue 1, 2 is Blue 2, 3 is Red 1, 4 is Red 2.
        int Powershots = 0; //1 is yes, 2 is no.
        int ShootGoals = 0; //1 is yes, 2 is no.
        int OnlyPark = 0; //1 is yes, 2 is no.
        int AreYouMoving = 0; //1 is yes, 2 is no.
        int Save = 0; //1 is yes, 2 is no.
        int DelayAndGo = 0; //1 is yes, 2 is no.
/*
23.5 Inches Between the strips.
 */
        double strafePreset = 0;
        double rotationGoal = autoChassis.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double originalRotation = rotationGoal;
        double shootWait = 0;
        double rotate = 0;
        double strafe = 0;
        double drive = 0;

        //All Constants For All Moves
        double moveandliftdrive = 0;
        double moverightstrafe = 0;
        double adrive = 0;
        double astrafe = 0;
        double bdrive = 0;
        double bstrafe = 0;
        double cdrive = 0;
        double cstrafe = 0;
        double powershotdrive = 0;
        double topgoaldrive = 0;
        double powershotstrafe = 0;
        double topgoalstrafe = 0;

        int isRotate = 0;
        int isStrafe = 0;
        int isDrive = 0;
        int launchCount = 0;
        int ringCount = 0;
        ElapsedTime MeasureWait = new ElapsedTime();
        double driveValue = 0;
        double strafeValue = 0;
        autoChassis.SetRotation(autoChassis.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        while (!IsMenuDone) {
            switch (menu) {
                case StartLocation:
                    telemetry.addLine("Blue1(A) Blue2(B) Red1(X) Red2(Y)?");
                    telemetry.update();
                    if (gamepad1.a) {
                        StartLocation = 1;
                        OnRed = false;
                        menu = AutoCode2NoDelayAndGoToMiddle.Menu.ButtonWaiter0;
                    } else if (gamepad1.b) {
                        StartLocation = 2;
                        OnRed = false;
                        menu = AutoCode2NoDelayAndGoToMiddle.Menu.ButtonWaiter0;
                    } else if (gamepad1.x) {
                        StartLocation = 3;
                        OnRed = true;
                        menu = AutoCode2NoDelayAndGoToMiddle.Menu.ButtonWaiter0;
                    } else if (gamepad1.y) {
                        StartLocation = 4;
                        OnRed = true;
                        menu = AutoCode2NoDelayAndGoToMiddle.Menu.ButtonWaiter0;
                    }
                    break;
                case ButtonWaiter0:
                    if ((StartLocation == 1) && (!gamepad1.a)) {
                        menu = AutoCode2NoDelayAndGoToMiddle.Menu.DelayAndGo;
                    } else if ((StartLocation == 2) && (!gamepad1.b)) {
                        menu = AutoCode2NoDelayAndGoToMiddle.Menu.DelayAndGo;
                    } else if ((StartLocation == 3) && (!gamepad1.x)) {
                        menu = AutoCode2NoDelayAndGoToMiddle.Menu.DelayAndGo;
                    } else if ((StartLocation == 4) && (!gamepad1.y)) {
                        menu = AutoCode2NoDelayAndGoToMiddle.Menu.DelayAndGo;
                    }
                    break;
                case DelayAndGo:
                    telemetry.addLine("Are you doing the Wobble Goals? If you aren't, then the robot will delay for 11 seconds and then go straight to the launch location. Yes (Y) No (X)");
                    telemetry.update();
                    if (gamepad1.x) {
                        DelayAndGo = 2;
                        menu = AutoCode2NoDelayAndGoToMiddle.Menu.ButtonWaiter1;
                    } else if (gamepad1.y) {
                        DelayAndGo = 1;
                        menu = AutoCode2NoDelayAndGoToMiddle.Menu.ButtonWaiter1;
                    }
                    break;
                case ButtonWaiter1:
                    if ((DelayAndGo == 2) && (!gamepad1.x)) {
                        menu = AutoCode2NoDelayAndGoToMiddle.Menu.Powershots;
                    } else if ((DelayAndGo == 1) && (!gamepad1.y)) {
                        menu = AutoCode2NoDelayAndGoToMiddle.Menu.Powershots;
                    }
                    break;
                case Powershots:
                    telemetry.addLine("Shoot Powershots? Yes(Y) No(X)");
                    telemetry.update();
                    if (gamepad1.x) {
                        Powershots = 2;
                        menu = AutoCode2NoDelayAndGoToMiddle.Menu.ButtonWaiter2;
                    } else if (gamepad1.y) {
                        Powershots = 1;
                        menu = AutoCode2NoDelayAndGoToMiddle.Menu.ButtonWaiter2;
                    }
                    break;
                case ButtonWaiter2:
                    if ((Powershots == 2) && (!gamepad1.x)) {
                        menu = AutoCode2NoDelayAndGoToMiddle.Menu.Goals;
                    } else if ((Powershots == 1) && (!gamepad1.y)) {
                        menu = AutoCode2NoDelayAndGoToMiddle.Menu.CheckForInvalid;
                    }
                    break;
                case Goals:
                    telemetry.addLine("Shoot into the top goal? Yes(Y) No(X)");
                    telemetry.update();
                    if (gamepad1.x) {
                        ShootGoals = 2;
                        menu = AutoCode2NoDelayAndGoToMiddle.Menu.ButtonWaiter3;
                    } else if (gamepad1.y) {
                        ShootGoals = 1;
                        menu = AutoCode2NoDelayAndGoToMiddle.Menu.ButtonWaiter3;
                    }
                    break;
                case ButtonWaiter3:
                    if ((ShootGoals == 2) & (!gamepad1.x)) {
                        menu = AutoCode2NoDelayAndGoToMiddle.Menu.OnlyPark;
                    } else if ((ShootGoals == 1) & (!gamepad1.y)) {
                        menu = AutoCode2NoDelayAndGoToMiddle.Menu.CheckForInvalid;
                    }
                    break;
                case OnlyPark:
                    telemetry.addLine("Would you like to park in the corner? Yes(Y) No(X)");
                    telemetry.update();
                    if (gamepad1.x) {
                        OnlyPark = 2;
                        menu = AutoCode2NoDelayAndGoToMiddle.Menu.ButtonWaiter4;
                    } else if (gamepad1.y) {
                        OnlyPark = 1;
                        menu = AutoCode2NoDelayAndGoToMiddle.Menu.ButtonWaiter4;
                    }
                    break;
                case ButtonWaiter4:
                    if ((OnlyPark == 2) & (!gamepad1.x)) {
                        menu = AutoCode2NoDelayAndGoToMiddle.Menu.AreYouMoving;
                    } else if ((OnlyPark == 1) & (!gamepad1.y)) {
                        menu = AutoCode2NoDelayAndGoToMiddle.Menu.CheckForInvalid;
                    }
                    break;
                case AreYouMoving:
                    telemetry.addLine("Would you like to stay put? Yes(Y) No(X) No will cause an error and kick you back to the beginning.");
                    telemetry.update();
                    if (gamepad1.x) {
                        AreYouMoving = 2;
                        menu = AutoCode2NoDelayAndGoToMiddle.Menu.ButtonWaiter5;
                    } else if (gamepad1.y) {
                        AreYouMoving = 1;
                        menu = AutoCode2NoDelayAndGoToMiddle.Menu.ButtonWaiter5;
                    }
                    break;
                case ButtonWaiter5:
                    if ((AreYouMoving == 2) & (!gamepad1.x)) {
                        menu = AutoCode2NoDelayAndGoToMiddle.Menu.CheckForInvalid;
                    } else if ((AreYouMoving == 1) & (!gamepad1.y)) {
                        menu = AutoCode2NoDelayAndGoToMiddle.Menu.CheckForInvalid;
                    }
                    break;
                case CheckForInvalid:
                    if (Powershots == 2 & ShootGoals == 2 & OnlyPark == 2 & AreYouMoving == 2) { menu = AutoCode2NoDelayAndGoToMiddle.Menu.Redo; }
                    else { menu = AutoCode2NoDelayAndGoToMiddle.Menu.AskIfDone; }
                    break;
                case AskIfDone:
                    telemetry.addLine("Would you like to save these changes? Yes(Y) No(X)");
                    telemetry.update();
                    if (gamepad1.x) {
                        Save = 2;
                        menu = AutoCode2NoDelayAndGoToMiddle.Menu.ButtonWaiter6;
                    } else if (gamepad1.y) {
                        Save = 1;
                        menu = AutoCode2NoDelayAndGoToMiddle.Menu.ButtonWaiter6;
                    }
                    break;
                case ButtonWaiter6:
                    if ((Save == 2) & (!gamepad1.x)) {
                        menu = AutoCode2NoDelayAndGoToMiddle.Menu.Redo;
                    } else if ((Save == 1) & (!gamepad1.y)) {
                        menu = AutoCode2NoDelayAndGoToMiddle.Menu.Save;
                    }
                    break;
                case Redo:
                    StartLocation = 0;
                    Powershots = 0;
                    ShootGoals = 0;
                    OnlyPark = 0;
                    AreYouMoving = 0;
                    Save = 0;
                    DelayAndGo = 0;
                    menu = AutoCode2NoDelayAndGoToMiddle.Menu.StartLocation;
                    break;
                case Save:
                    if (DelayAndGo == 2) {
                        driveOpState = AutoCode2NoDelayAndGoToMiddle.OperState.Delayer;
                        DoneMeasuring = true;
                    }
                    switch (StartLocation) {
                        case 1:
                            moveandliftdrive = -19;
                            moverightstrafe = -6.5;
                            adrive = -28;
                            astrafe = 7;
                            bdrive = -51;
                            bstrafe = -9.5;
                            cdrive = -72;
                            cstrafe = 7;
                            if (DelayAndGo == 2) {
                                if (Powershots == 1) {
                                    powershotdrive = 0;
                                    powershotstrafe = 0;
                                }
                                
                                if (ShootGoals == 1) {
                                    topgoaldrive = 56.5;
                                    topgoalstrafe = 22.5;
                                }
                            }
                            break;

                        case 2:
                            moveandliftdrive = -19;
                            moverightstrafe = 17;
                            adrive = -28;
                            astrafe = 30.5;
                            bdrive = -51;
                            bstrafe = 14;
                            cdrive = -72;
                            cstrafe = 30.5;
                            if (DelayAndGo == 2) {
                                if (Powershots == 1) {
                                    powershotdrive = 0;
                                    powershotstrafe = 0;
                                }
                                
                                if (ShootGoals == 1) {
                                    topgoaldrive = 56.5;
                                    topgoalstrafe = -4;
                                }
                            }
                            break;

                        case 3:
                            moveandliftdrive = -19;
                            moverightstrafe = -6.5;
                            adrive = -28;
                            astrafe = -30.5;
                            bdrive = -51;
                            bstrafe = -14;
                            cdrive = -72;
                            cstrafe = -30.5;
                            if (DelayAndGo == 2) {
                                if (Powershots == 1) {
                                    powershotdrive = 0;
                                    powershotstrafe = 0;
                                }
                                
                                if (ShootGoals == 1) {
                                    topgoaldrive = 56.5;
                                    topgoalstrafe = 22.5;
                                }
                            }
                            break;

                        case 4:
                            moveandliftdrive = -19;
                            moverightstrafe = 17;
                            adrive = -28;
                            astrafe = -7;
                            bdrive = -51;
                            bstrafe = 9.5;
                            cdrive = -72;
                            cstrafe = -7;
                            if (DelayAndGo == 2) {
                                if (Powershots == 1) {
                                    powershotdrive = 56.5;
                                    powershotstrafe = -4;
                                }
                                
                                if (ShootGoals == 1) {
                                    topgoaldrive = 56.5;
                                    topgoalstrafe = -4;
                                }
                            }
                            break;

                    }
                    telemetry.addLine("Choices have been saved. You may now tell the ref you are ready.");
                    telemetry.update();
                    IsMenuDone = true;
                    break;
            }
        }

        waitForStart();
        servoTimer.reset();
        launcher.Reload();
        launcher.LauncherToggle();
/*                    if (autoChassis.MoveToLocation() == true) {
                        telemetry.addLine("done");
                    }
*/
        while (opModeIsActive()) {

            autoChassis.SetRotation(autoChassis.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            launcher.LauncherRun(1);
            if (!DoneMeasuring) { ring.MeasureDistance(); }
            telemetry.addData("where you are in strafe", Math.abs(autoChassis.strafePreset - autoChassis.trueStrafe));
            telemetry.addData("driveopstate", driveOpState);
            telemetry.addData("IMPORTANT, DRIVE PRESET", autoChassis.drivePreset);
            telemetry.addData("IMPORTANT, STRAFE PRESET", autoChassis.strafePreset);
            telemetry.addData("Launch Count", launchCount);
            switch (driveOpState) {
                case FIRSTMOVE:
                    telemetry.addLine("FIRSTMOVE");
                    lift.MoveServo(1);
                    originalRotation = autoChassis.zAngle;

                    if (servoTimer.time() >= 2) {
                        lift.MoveServo(0);
                        driveOpState = AutoCode2NoDelayAndGoToMiddle.OperState.PREPMOVEANDLIFT;
                    }
                    break;

                case RESETTIMER:
                    MeasureWait.reset();
                    driveOpState = AutoCode2NoDelayAndGoToMiddle.OperState.MEASURE;
                    break;

                case Delayer:
                    if (servoTimer.time() >= 0.25) {
                        if (Powershots == 1) {
                            driveOpState = AutoCode2NoDelayAndGoToMiddle.OperState.PrepMoveToPowerShots;
                        } else if (ShootGoals == 1) {
                            driveOpState = AutoCode2NoDelayAndGoToMiddle.OperState.PrepMoveToGoals;
                            servoTimer.reset();
                        }
                    }
                    break;

                case PREPMOVEANDLIFT:
                    autoChassis.SetAxisMovement();
                    autoChassis.ZeroEncoders();
                    autoChassis.SetAxisMovement();
                    autoChassis.SetPresetMovement(-19., 0.5, 0, 0.000000000000001, autoChassis.zAngle);
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
                        driveOpState = AutoCode2NoDelayAndGoToMiddle.OperState.LIFTUP;
                        servoTimer.reset();
                    }
                    break;
                case LIFTUP:
                    lift.MoveServo(-1);
                    if (servoTimer.time() >= 2) {
                        lift.MoveServo(0);
                        driveOpState = AutoCode2NoDelayAndGoToMiddle.OperState.PREPMOVERIGHT;
                    }
                    break;
                case PREPMOVERIGHT:
                    autoChassis.SetAxisMovement();
                    autoChassis.ZeroEncoders();
                    autoChassis.SetAxisMovement();
                    autoChassis.SetPresetMovement(0, 0,moverightstrafe, 0.4, autoChassis.zAngle);
                    servoTimer.reset();
                    driveOpState = OperState.MOVERIGHT;
                    break;

                case MOVERIGHT:
                    telemetry.addData("front left wheel", autoChassis.frontLeft + " = " + (1 * autoChassis.drive) + " + " + (1 * autoChassis.strafe) + " + " + (-1 * autoChassis.rotation));
                    telemetry.addData("back left wheel", autoChassis.backLeft + " = " + (-1 * autoChassis.drive) + " + " + (1 * autoChassis.strafe) + " + " + (1 * autoChassis.rotation));
                    telemetry.addData("front right wheel", autoChassis.frontRight + " = " + (1 * autoChassis.drive) + " + " + (-1 * autoChassis.strafe) + " + " + (1 * autoChassis.rotation));
                    telemetry.addData("back right wheel", autoChassis.backRight + " = " + (1 * autoChassis.drive) + " + " + (1 * autoChassis.strafe) + " + " + (1 * autoChassis.rotation));
                    if (autoChassis.MoveToLocation() == true) {
                        driveOpState = OperState.MEASURE;
                        MeasureWait.reset();
                    }
                    break;
                case MEASURE:
                    if (MeasureWait.time(TimeUnit.SECONDS) >= 0.6) {
                        ringCount = ring.RingHeight();
                        driveOpState = OperState.PREPMOVEBACK;
                    }
                    break;
                case PREPMOVEBACK:
                    autoChassis.SetAxisMovement();
                    autoChassis.ZeroEncoders();
                    autoChassis.SetAxisMovement();
                    autoChassis.SetPresetMovement(0, 0, -moverightstrafe, 0.4, autoChassis.zAngle);
                    driveOpState = OperState.MOVEBACK;
                    break;
                case MOVEBACK:
                    telemetry.addData("front left wheel", autoChassis.frontLeft + " = " + (1 * autoChassis.drive) + " + " + (1 * autoChassis.strafe) + " + " + (-1 * autoChassis.rotation));
                    telemetry.addData("back left wheel", autoChassis.backLeft + " = " + (-1 * autoChassis.drive) + " + " + (1 * autoChassis.strafe) + " + " + (1 * autoChassis.rotation));
                    telemetry.addData("front right wheel", autoChassis.frontRight + " = " + (1 * autoChassis.drive) + " + " + (-1 * autoChassis.strafe) + " + " + (1 * autoChassis.rotation));
                    telemetry.addData("back right wheel", autoChassis.backRight + " = " + (1 * autoChassis.drive) + " + " + (1 * autoChassis.strafe) + " + " + (1 * autoChassis.rotation));
                    if (autoChassis.MoveToLocation() == true) {
                        servoTimer.reset();
                        driveOpState = OperState.LIFTDOWN;
                    }
                    break;

                case LIFTDOWN:
                    lift.MoveServo(1);
                    if (servoTimer.time() >= 2) {
                        lift.MoveServo(0);
                        driveOpState = AutoCode2NoDelayAndGoToMiddle.OperState.DECIDE;
                    }
                    break;
                case DECIDE:
                    if (ringCount == 0) {
                        if (Powershots == 1) {
                            powershotdrive = 1;
                            powershotstrafe = 1;
                            if (!OnRed) {
                                powershotstrafe = -powershotstrafe;
                            }
                        }
                        if (ShootGoals == 1) {
                            topgoaldrive = 1;
                            topgoalstrafe = 1;
                            if (!OnRed) {
                                topgoalstrafe = -topgoalstrafe;
                            }
                        }
                        driveOpState = AutoCode2NoDelayAndGoToMiddle.OperState.PREPA;
                    } else if (ringCount == 1) {
                        if (Powershots == 1) {
                            powershotdrive = 1;
                            powershotstrafe = 1;
                            if (!OnRed) {
                                powershotstrafe = -powershotstrafe;
                            }
                        }
                        if (ShootGoals == 1) {
                            topgoaldrive = 1;
                            topgoalstrafe = 1;
                            if (!OnRed) {
                                topgoalstrafe = -topgoalstrafe;
                            }
                        }
                        driveOpState = AutoCode2NoDelayAndGoToMiddle.OperState.PREPB;
                    } else if (ringCount == 4) {
                        if (Powershots == 1) {
                            powershotdrive = 1;
                            powershotstrafe = 1;
                            if (!OnRed) {
                                powershotstrafe = -powershotstrafe;
                            }
                        }
                        if (ShootGoals == 1) {
                            topgoaldrive = 1;
                            topgoalstrafe = 1;
                            if (!OnRed) {
                                topgoalstrafe = -topgoalstrafe;
                            }
                        }
                        driveOpState = AutoCode2NoDelayAndGoToMiddle.OperState.PREPC;
                    }
                    DoneMeasuring = true;
                    break;
                case PREPA:
                    autoChassis.SetAxisMovement();
                    autoChassis.ZeroEncoders();
                    autoChassis.SetAxisMovement();
                    autoChassis.SetPresetMovement(adrive, 1, astrafe, .4, autoChassis.zAngle);
                    driveOpState = OperState.GoToTargetZone;
                    break;
                case PREPB:
                    autoChassis.SetAxisMovement();
                    autoChassis.ZeroEncoders();
                    autoChassis.SetAxisMovement();
                    autoChassis.SetPresetMovement(bdrive, 1, bstrafe, .4, autoChassis.zAngle);
                    driveOpState = OperState.GoToTargetZone;
                    break;
                case PREPC:
                    autoChassis.SetAxisMovement();
                    autoChassis.ZeroEncoders();
                    autoChassis.SetAxisMovement();
                    autoChassis.SetPresetMovement(cdrive, 1, cstrafe, .4, autoChassis.zAngle);
                    driveOpState = OperState.GoToTargetZone;
                    break;
                case GoToTargetZone:
                    autoChassis.rotationPreset -= 0.105;
                    if (autoChassis.MoveToLocation() == true) {
                        driveOpState = AutoCode2NoDelayAndGoToMiddle.OperState.NextLocation;
                    }
                    break;

                case NextLocation:
                    if (Powershots == 1) {
                        driveOpState = AutoCode2NoDelayAndGoToMiddle.OperState.PrepMoveToPowerShots;
                    } else if (ShootGoals == 1) {
                        driveOpState = AutoCode2NoDelayAndGoToMiddle.OperState.PrepMoveToGoals;
                    }
                    break;
                case PrepMoveToPowerShots:
                    autoChassis.SetAxisMovement();
                    autoChassis.ZeroEncoders();
                    autoChassis.SetAxisMovement();
                    autoChassis.SetPresetMovement(powershotdrive, 1, powershotstrafe, .4, autoChassis.zAngle);
                    driveOpState = AutoCode2NoDelayAndGoToMiddle.OperState.MoveToPowerShots;
                    launcher.LauncherToggle();
                    break;
                case MoveToPowerShots:
                    if (autoChassis.MoveToLocation() == true) {
                        driveOpState = AutoCode2NoDelayAndGoToMiddle.OperState.PrepSpinAround;
                    }
                    break;
                case PrepMoveToGoals:
                    autoChassis.SetAxisMovement();
                    autoChassis.ZeroEncoders();
                    autoChassis.SetAxisMovement();
                    autoChassis.SetPresetMovement(topgoaldrive, 1.2, topgoalstrafe, .4, autoChassis.zAngle);
                    driveOpState = AutoCode2NoDelayAndGoToMiddle.OperState.MoveToGoals;
                    break;
                case MoveToGoals:
                    if (autoChassis.MoveToLocation() == true) { driveOpState = AutoCode2NoDelayAndGoToMiddle.OperState.Launch; }
                    break;
                case PrepSpinAround:
                    if ((Math.abs(autoChassis.zAngle - (originalRotation-180)) >= 2)) {
                        autoChassis.SetMotors(0, 0, autoChassis.CorrectRotation(autoChassis.zAngle, (originalRotation-180)));
                        autoChassis.Drive();
                        autoChassis.SetAxisMovement();
                        autoChassis.ZeroEncoders();
                        autoChassis.SetAxisMovement();
                        rotationGoal = autoChassis.zAngle;
                    }
                    else {
                        driveOpState = AutoCode2NoDelayAndGoToMiddle.OperState.Launch;
                    }
                    break;
                case Launch:
                    if (launchCount <= 2 && ((ShootGoals == 1 && servoTimer.time() >= 1.3) || (Powershots == 1 && servoTimer.time() >= 0.125))) {
                        launcher.Shoot();
                        launchCount++;
                        if (ShootGoals == 1) {
                            driveOpState = AutoCode2NoDelayAndGoToMiddle.OperState.Reload;
                            MeasureWait.reset();
                        }
                        else if (Powershots == 1 && launchCount < 2) { driveOpState = AutoCode2NoDelayAndGoToMiddle.OperState.PrepStrafeLeft; }
                        MeasureWait.reset();
                    }
                    else if (launchCount > 2) { driveOpState = AutoCode2NoDelayAndGoToMiddle.OperState.PrepLaunchPark; }
                    break;
                case Reload:
                    if (MeasureWait.time(TimeUnit.SECONDS) >= 1.3 ) {
                        launcher.Reload();
                        driveOpState = AutoCode2NoDelayAndGoToMiddle.OperState.Launch;
                        servoTimer.reset();
                    }
                    break;
                case PrepStrafeLeft:
                    autoChassis.SetAxisMovement();
                    autoChassis.ZeroEncoders();
                    autoChassis.SetAxisMovement();
                    autoChassis.SetPresetMovement(13, 1, 0, .4, autoChassis.zAngle);
                    driveOpState = AutoCode2NoDelayAndGoToMiddle.OperState.StrafeLeft;
                    break;
                case StrafeLeft:
                    if (autoChassis.MoveToLocation() == true) {
                        driveOpState = AutoCode2NoDelayAndGoToMiddle.OperState.Reload;
                    }
                    break;
                case PrepLaunchPark:
                    autoChassis.SetAxisMovement();
                    autoChassis.ZeroEncoders();
                    autoChassis.SetAxisMovement();
                    autoChassis.SetPresetMovement(13, 1, 20, .41, autoChassis.zAngle);
                    driveOpState = AutoCode2NoDelayAndGoToMiddle.OperState.LaunchPark;
                case LaunchPark:
                    if (autoChassis.MoveToLocation() == true) {
                        telemetry.addLine("done");
                    }
                    break;
                case THIRDMOVE:
                    telemetry.addLine("THIRDMOVE");
                    lift.MoveServo(-1);

                    if (servoTimer.time() >= 2) {
                        lift.MoveServo(0);
                        driveOpState = AutoCode2NoDelayAndGoToMiddle.OperState.STARTLAUNCHER;
                    }
                    break;

                case STARTLAUNCHER:
                    launcher.LauncherToggle();
                    driveOpState = AutoCode2NoDelayAndGoToMiddle.OperState.FOURTHMOVESETUP;

                case SHOOT1:
                    if (shootWait < 6) {
                        if (servoTimer.time() > 0.5) {
                            driveOpState = AutoCode2NoDelayAndGoToMiddle.OperState.firsttimer;
                        }
                    } else {
                        driveOpState = AutoCode2NoDelayAndGoToMiddle.OperState.FIFTHMOVESETUP;
                    }

                    break;
                case firsttimer:
                    servoTimer.reset();
                    driveOpState = AutoCode2NoDelayAndGoToMiddle.OperState.Load;
                    break;

                case Load:
                    launcher.Shoot();
                    if (servoTimer.time() >= 0.15) {
                        driveOpState = AutoCode2NoDelayAndGoToMiddle.OperState.secondtimer;
                    }
                    break;

                case secondtimer:
                    servoTimer.reset();
                    driveOpState = AutoCode2NoDelayAndGoToMiddle.OperState.ResetPosition;
                    break;

                case ResetPosition:
                    launcher.Reload();
                    if (servoTimer.time() >= 0.15) {
                        shootWait += 1;
                        servoTimer.reset();
                        driveOpState = AutoCode2NoDelayAndGoToMiddle.OperState.SHOOT1;
                    }
                    break;

            }
            telemetry.update();
        }
    }
}
