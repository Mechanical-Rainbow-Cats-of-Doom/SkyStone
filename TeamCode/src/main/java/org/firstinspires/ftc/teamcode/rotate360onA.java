package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

//Created by mostly Patrick, partly Ethan
@Autonomous
public class rotate360onA extends LinearOpMode {

    private Blinker Control_Hub;
    private Blinker expansion_Hub_2;

    //motors

    private BNO055IMU imu;

    private DigitalChannel switch_;

    ElapsedTime servoTimer = new ElapsedTime();

    enum OperState {
        SETUP,
        rotate,
        DRIVEANDROTATE
    }

    @Override
    public void runOpMode() {
        LauncherCode.Launcher launcher = new LauncherCode.Launcher();
        LifterCode.Lifter lift = new LifterCode.Lifter();
        ChassisMovementCode.Chassis autoChassis = new ChassisMovementCode.Chassis();
        rotate360onA.OperState driveOpState = rotate360onA.OperState.SETUP;


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
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        autoChassis.imu.initialize(parameters);
        double drivePreset = 0;
        double rotationGoal = autoChassis.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double originalRotation = rotationGoal;
        double shootWait = 0;
        double initialLeft = 0;
        double initialBack = 0;
        double initialRight = 0;
        double directionSwitch = -1;

        double AveragedArray;
        double totalLeft = 0;
        double totalBack = 0;
        double totalRight =0;
        ElapsedTime driveTimer = new ElapsedTime();
        int index = 0;
        int ArraySize = 500; //Allows for easy way to change the size of the array that affects all of the code
        double[] movementArrayLeft = new double [ArraySize];
        double[] movementArrayBack = new double [ArraySize];
        double[] movementArrayRight = new double [ArraySize];


        autoChassis.SetRotation(autoChassis.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        waitForStart();
        servoTimer.reset();

        while (opModeIsActive()) {
            autoChassis.SetRotation(autoChassis.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            launcher.LauncherRun(1);

            switch (driveOpState) {
                case SETUP:
                    telemetry.addData("Right Encoder", autoChassis.back_right_wheel.getCurrentPosition());
                    telemetry.addData("Left Encoder", -autoChassis.front_right_wheel.getCurrentPosition());
                    telemetry.addData("Back Encoder", autoChassis.front_left_wheel.getCurrentPosition());
                    telemetry.addData("Initial Right Encoder", initialRight);
                    telemetry.addData("Initial Left Encoder", initialLeft);
                    telemetry.addData("Initial Back Encoder", initialBack);
                    telemetry.addData("Right Encoder Difference", autoChassis.back_right_wheel.getCurrentPosition()-initialRight);
                    telemetry.addData("Left Encoder Difference", -autoChassis.front_right_wheel.getCurrentPosition()-initialLeft);
                    telemetry.addData("Back Encoder Difference", autoChassis.front_left_wheel.getCurrentPosition()-initialBack);
                    telemetry.addData("Right Encoder Average Difference", totalRight / (index-1));
                    telemetry.addData("Left Encoder Average Difference", totalLeft / (index-1));
                    telemetry.addData("Back Encoder Average Difference", totalBack / (index-1));
                    telemetry.addData("Right Encoder Multiplier", (totalBack / (index-1)) / (totalRight / (index-1)) );
                    telemetry.addData("Left Encoder Multiplier", (totalBack / (index-1)) / (totalLeft / (index-1)) );
                    telemetry.addData("Back Encoder Multiplier" , 1);

                    if (gamepad1.a) {
                        if (index >= (ArraySize - 1)) {
                            index = 0;
                        }
                        else {
                            index++;
                        }
                        movementArrayLeft[index] = -autoChassis.front_right_wheel.getCurrentPosition()-initialLeft;
                        movementArrayBack[index] = autoChassis.front_left_wheel.getCurrentPosition()-initialBack;
                        movementArrayRight[index] = autoChassis.back_right_wheel.getCurrentPosition()-initialRight;

                        initialLeft = -autoChassis.front_right_wheel.getCurrentPosition();
                        initialBack = autoChassis.front_left_wheel.getCurrentPosition();
                        initialRight = autoChassis.back_right_wheel.getCurrentPosition();
                        rotationGoal = autoChassis.zAngle;
                        driveOpState = rotate360onA.OperState.rotate;
                    }
                    if (gamepad1.right_trigger != 0) {
                        initialLeft = -autoChassis.front_right_wheel.getCurrentPosition();
                        initialBack = autoChassis.front_left_wheel.getCurrentPosition();
                        initialRight = autoChassis.back_right_wheel.getCurrentPosition();
                        rotationGoal = autoChassis.zAngle;
                        driveTimer.reset();
                        directionSwitch = directionSwitch * -1;
                        driveOpState = rotate360onA.OperState.DRIVEANDROTATE;
                    }
                    break;

                case DRIVEANDROTATE:
                    if (driveTimer.time() <= 8) {
                        telemetry.addLine("Drive and Rotate");
                        autoChassis.SetAxisMovement();
                        //double drive = -this.gamepad1.left_stick_y;
                        //strafe = this.gamepad1.left_stick_x;
                        double rotate = -this.gamepad1.right_stick_x;

                        autoChassis.SetPresetAxis();

                        autoChassis.SetMotors(0.5 * directionSwitch, 0, 0);
                        autoChassis.Drive();
                        autoChassis.SetTrueAxis();
                    }
                    else {
                        autoChassis.front_left_wheel.setPower(-0.01);
                        autoChassis.front_right_wheel.setPower(-0.01);
                        autoChassis.back_right_wheel.setPower(-0.01);
                        autoChassis.back_left_wheel.setPower(-0.01);
                        if (index >= (ArraySize - 1)) {
                            index = 0;
                        }
                        else {
                            index++;
                        }
                        //movementArrayLeft[index] = Math.abs((-autoChassis.front_right_wheel.getCurrentPosition()-initialLeft)*autoChassis.leftEncoderMultiplier);
                        //movementArrayBack[index] = Math.abs((autoChassis.front_left_wheel.getCurrentPosition()-initialBack)*autoChassis.backEncoderMultiplier);
                        //movementArrayRight[index] = Math.abs((autoChassis.back_right_wheel.getCurrentPosition()-initialRight)*autoChassis.rightEncoderMultiplier);
                        totalLeft = 0;
                        totalBack = 0;
                        totalRight = 0;
                        for (int i = 0; i <= (ArraySize - 1); i++) {
                            totalLeft = totalLeft + movementArrayLeft[i];
                            totalRight = totalRight + movementArrayRight[i];
                            totalBack = totalBack + movementArrayBack[i];
                        }
                    }
                    break;

                case rotate:
                    telemetry.addLine("Rotate");
                    if ((Math.abs(autoChassis.zAngle - (rotationGoal - 180)) >= 2)) {
                        autoChassis.SetMotors(0, 0, autoChassis.CorrectRotation(autoChassis.zAngle, (rotationGoal - 180)));
                        autoChassis.Drive();
                        autoChassis.SetAxisMovement();
                        autoChassis.ZeroEncoders();
                        autoChassis.SetAxisMovement();

                    } else {
                        autoChassis.front_left_wheel.setPower(-0.01);
                        autoChassis.front_right_wheel.setPower(-0.01);
                        autoChassis.back_right_wheel.setPower(-0.01);
                        autoChassis.back_left_wheel.setPower(-0.01);
                        if (index >= (ArraySize - 1)) {
                            index = 0;
                        }
                        else {
                            index++;
                        }
                        //movementArrayLeft[index] = (-autoChassis.front_right_wheel.getCurrentPosition()-initialLeft)*autoChassis.leftEncoderMultiplier;
                        //movementArrayBack[index] = (autoChassis.front_left_wheel.getCurrentPosition()-initialBack)*autoChassis.backEncoderMultiplier;
                        //movementArrayRight[index] = (autoChassis.back_right_wheel.getCurrentPosition()-initialRight)*autoChassis.rightEncoderMultiplier;
                        totalLeft = 0;
                        totalBack = 0;
                        totalRight = 0;
                        for (int i = 0; i <= (ArraySize - 1); i++) {
                            totalLeft = totalLeft + movementArrayLeft[i];
                            totalRight = totalRight + movementArrayRight[i];
                            totalBack = totalBack + movementArrayBack[i];
                        }
                        driveOpState = rotate360onA.OperState.SETUP;
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
 */


            }/*cdg*/

            telemetry.update();
        }
    }
}
