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
public class AutoCode2 extends LinearOpMode {

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
        A,
        B,
        C
    }
    enum Menu  {
        StartLocation,
        ButtonWaiter,
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
        AutoCode2.OperState driveOpState = AutoCode2.OperState.FIRSTMOVE;
        AutoCode2.Menu menu = AutoCode2.Menu.StartLocation;
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

        //menu variables
        boolean IsMenuDone = false;
        int StartLocation = 0; //1 is Blue 1, 2 is Blue 2, 3 is Red 1, 4 is Red 2.
        int Powershots = 0; //1 is yes, 2 is no.
        int ShootGoals = 0; //1 is yes, 2 is no.
        int OnlyPark = 0; //1 is yes, 2 is no.
        int AreYouMoving = 0; //1 is yes, 2 is no.
        int Save = 0; //1 is yes, 2 is no.
/*
23.5 Inches Between the strips.
 */
        double strafePreset = 0;
        double rotationGoal = autoChassis.imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES).firstAngle;
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

        int isRotate = 0;
        int isStrafe = 0;
        int isDrive = 0;
        int ringCount = 0;
        ElapsedTime MeasureWait = new ElapsedTime();
        double driveValue = 0;
        double strafeValue = 0;
        autoChassis.SetRotation(autoChassis.imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES).firstAngle);
        while (!IsMenuDone) {
            switch (menu) {
                case StartLocation:
                    telemetry.addLine("Blue1(A) Blue2(B) Red1(X) Red2(Y)?");
                    telemetry.update();
                    if (gamepad1.a) {
                        StartLocation = 1;
                        menu = AutoCode2.Menu.ButtonWaiter;
                    }
                    else if (gamepad1.b) {
                        StartLocation = 2;
                        menu = AutoCode2.Menu.ButtonWaiter;
                    }
                    else if (gamepad1.x) {
                        StartLocation = 3;
                        menu = AutoCode2.Menu.ButtonWaiter;
                    }
                    else if (gamepad1.y) {
                        StartLocation = 4;
                        menu = AutoCode2.Menu.ButtonWaiter;
                    }
                    break;
                case ButtonWaiter:
                    if ((StartLocation == 1) & (!gamepad1.a)) { menu = AutoCode2.Menu.Powershots; }
                    else if ((StartLocation == 2) & (!gamepad1.b)) { menu = AutoCode2.Menu.Powershots; }
                    else if ((StartLocation == 3) & (!gamepad1.x)) { menu = AutoCode2.Menu.Powershots; }
                    else if ((StartLocation == 4) & (!gamepad1.y)) { menu = AutoCode2.Menu.Powershots; }
                    break;
                case Powershots:
                    telemetry.addLine("Shoot Powershots? Yes(Y) No(X)");
                    telemetry.update();
                    if (gamepad1.x) {
                        Powershots = 2;
                        menu = AutoCode2.Menu.ButtonWaiter2;
                    }
                    else if (gamepad1.y) {
                        Powershots = 1;
                        menu = AutoCode2.Menu.ButtonWaiter2;
                    }
                    break;
                case ButtonWaiter2:
                    if ((Powershots == 2) & (!gamepad1.x)) { menu = AutoCode2.Menu.Goals; }
                    else if ((Powershots == 1) & (!gamepad1.y)) { menu = AutoCode2.Menu.CheckForInvalid; }
                    break;
                case Goals:
                    telemetry.addLine("Shoot into Goal? Yes(Y) No(X)");
                    telemetry.update();
                    if (gamepad1.x) {
                        ShootGoals = 2;
                        menu = AutoCode2.Menu.ButtonWaiter3;
                    }
                    else if (gamepad1.y) {
                        ShootGoals = 1;
                        menu = AutoCode2.Menu.ButtonWaiter3;
                    }
                    break;
                case ButtonWaiter3:
                    if ((ShootGoals == 2) & (!gamepad1.x)) { menu = AutoCode2.Menu.OnlyPark; }
                    else if ((ShootGoals == 1) & (!gamepad1.y)) { menu = AutoCode2.Menu.CheckForInvalid; }
                    break;
                case OnlyPark:
                    telemetry.addLine("Would you like to park in the corner? Yes(Y) No(X)");
                    telemetry.update();
                    if (gamepad1.x) {
                        OnlyPark = 2;
                        menu = AutoCode2.Menu.ButtonWaiter4;
                    }
                    else if (gamepad1.y) {
                        OnlyPark = 1;
                        menu = AutoCode2.Menu.ButtonWaiter4;
                    }
                    break;
                case ButtonWaiter4:
                    if ((OnlyPark == 2) & (!gamepad1.x)) { menu = AutoCode2.Menu.AreYouMoving; }
                    else if ((OnlyPark == 1) & (!gamepad1.y)) { menu = AutoCode2.Menu.CheckForInvalid; }
                    break;
                case AreYouMoving:
                    telemetry.addLine("Would you like to stay put? Yes(Y) No(X) No will cause an error and kick you back to the beginning.");
                    telemetry.update();
                    if (gamepad1.x) {
                        AreYouMoving = 2;
                        menu = AutoCode2.Menu.ButtonWaiter5;
                    }
                    else if (gamepad1.y) {
                        AreYouMoving = 1;
                        menu = AutoCode2.Menu.ButtonWaiter5;
                    }
                    break;
                case ButtonWaiter5:
                    if ((AreYouMoving == 2) & (!gamepad1.x)) { menu = AutoCode2.Menu.CheckForInvalid; }
                    else if ((AreYouMoving == 1) & (!gamepad1.y)) { menu = AutoCode2.Menu.CheckForInvalid; }
                    break;
                case CheckForInvalid:
                    if (Powershots == 2 & ShootGoals == 2 & OnlyPark == 2 & AreYouMoving == 2) {
                        StartLocation = 0;
                        Powershots = 0;
                        ShootGoals = 0;
                        OnlyPark = 0;
                        AreYouMoving = 0;
                        menu = AutoCode2.Menu.StartLocation;
                    }
                    else { menu = AutoCode2.Menu.AskIfDone; }
                    break;
                case AskIfDone:
                    telemetry.addLine("Would you like to save these changes? Yes(Y) No(X)");
                    telemetry.update();
                    if (gamepad1.x) {
                        Save = 2;
                        menu = AutoCode2.Menu.ButtonWaiter6;
                    }
                    else if (gamepad1.y) {
                        Save = 1;
                        menu = AutoCode2.Menu.ButtonWaiter6;
                    }
                    break;
                case ButtonWaiter6:
                    if ((Save == 2) & (!gamepad1.x)) { menu = AutoCode2.Menu.Redo; }
                    else if ((Save == 1) & (!gamepad1.y)) { menu = AutoCode2.Menu.Save; }
                    break;
                case Redo:
                    StartLocation = 0;
                    Powershots = 0;
                    ShootGoals = 0;
                    OnlyPark = 0;
                    AreYouMoving = 0;
                    Save = 0;
                    menu = AutoCode2.Menu.StartLocation;
                    break;
                case Save:
                    switch (StartLocation) {
                        case 1:
                            moveandliftdrive = -19;
                            moverightstrafe = -6.5;
                            adrive = -38.5;
                            astrafe = 7;
                            bdrive = -61;
                            bstrafe = -9.5;
                            cdrive = -82;
                            cstrafe = 7;
                            break;

                        case 2:
                            moveandliftdrive = -19;
                            moverightstrafe = 17;
                            adrive = -38.5;
                            astrafe = 30.5;
                            bdrive = -61;
                            bstrafe = 14;
                            cdrive = -82;
                            cstrafe = 30.5;
                            break;

                        case 3:
                            moveandliftdrive = -19;
                            moverightstrafe = -6.5;
                            adrive = -38.5;
                            astrafe = -30.5;
                            bdrive = -61;
                            bstrafe = -14;
                            cdrive = -82;
                            cstrafe = -30.5;
                            break;

                        case 4:
                            moveandliftdrive = -19;
                            moverightstrafe = 17;
                            adrive = -38.5;
                            astrafe = -7;
                            bdrive = -61;
                            bstrafe = 9.5;
                            cdrive = -82;
                            cstrafe = -7;
                            break;

                    }
                    telemetry.addLine("Choices have been saved. You may now tell the ref you are ready.");
                    telemetry.addData("Sum of Variables (If this is 0, something went very wrong. Please restart the robot if this is the case.)", StartLocation + Powershots + ShootGoals + OnlyPark + AreYouMoving);
                    telemetry.update();
                    IsMenuDone = true;
                    break;
            }

        }

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
            telemetry.addData("where you are in strafe", Math.abs(autoChassis.strafePreset - autoChassis.trueStrafe));
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
                    autoChassis.SetPresetMovement(moveandliftdrive, 0.5, 0, 0, autoChassis.zAngle);
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
                        driveOpState = AutoCode2.OperState.LIFTUP;
                        servoTimer.reset();
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
                    autoChassis.SetPresetMovement(0, 0, moverightstrafe, 0.4, autoChassis.zAngle);
                    servoTimer.reset();
                    driveOpState = OperState.MOVERIGHT;
                    break;
                case MOVERIGHT:
                    telemetry.addData("front left wheel", autoChassis.frontLeft + " = "+(1*autoChassis.drive) +" + "+(1*autoChassis.strafe)+" + "+(-1*autoChassis.rotation));
                    telemetry.addData("back left wheel", autoChassis.backLeft + " = "+(-1*autoChassis.drive) +" + "+(1*autoChassis.strafe)+" + "+(1*autoChassis.rotation));
                    telemetry.addData("front right wheel", autoChassis.frontRight + " = "+(1*autoChassis.drive) +" + "+(-1*autoChassis.strafe)+" + "+(1*autoChassis.rotation));
                    telemetry.addData("back right wheel", autoChassis.backRight + " = "+(1*autoChassis.drive) +" + "+(1*autoChassis.strafe)+" + "+(1*autoChassis.rotation));
                    if (autoChassis.MoveToLocation() == true) {
                        driveOpState = OperState.MEASURE;
                        MeasureWait.reset();
                    }
                    break;
                case MEASURE:
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
                    autoChassis.SetPresetMovement(0, 0, -moverightstrafe, 0.4, autoChassis.zAngle);
                    driveOpState = OperState.MOVEBACK;
                    break;
                case MOVEBACK:
                    telemetry.addData("front left wheel", autoChassis.frontLeft + " = "+(1*autoChassis.drive) +" + "+(1*autoChassis.strafe)+" + "+(-1*autoChassis.rotation));
                    telemetry.addData("back left wheel", autoChassis.backLeft + " = "+(-1*autoChassis.drive) +" + "+(1*autoChassis.strafe)+" + "+(1*autoChassis.rotation));
                    telemetry.addData("front right wheel", autoChassis.frontRight + " = "+(1*autoChassis.drive) +" + "+(-1*autoChassis.strafe)+" + "+(1*autoChassis.rotation));
                    telemetry.addData("back right wheel", autoChassis.backRight + " = "+(1*autoChassis.drive) +" + "+(1*autoChassis.strafe)+" + "+(1*autoChassis.rotation));
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
                    autoChassis.SetPresetMovement(adrive, 1, astrafe, .4, autoChassis.zAngle+2);
                    driveOpState = OperState.A;
                    break;
                case PREPB:
                    autoChassis.SetAxisMovement();
                    autoChassis.ZeroEncoders();
                    autoChassis.SetAxisMovement();
                    autoChassis.SetPresetMovement(bdrive, 1, bstrafe,.4, autoChassis.zAngle+2);
                    driveOpState = OperState.B;
                    break;
                case PREPC:
                    autoChassis.SetAxisMovement();
                    autoChassis.ZeroEncoders();
                    autoChassis.SetAxisMovement();
                    autoChassis.SetPresetMovement(cdrive, 1,  cstrafe, .4, autoChassis.zAngle+2);
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
