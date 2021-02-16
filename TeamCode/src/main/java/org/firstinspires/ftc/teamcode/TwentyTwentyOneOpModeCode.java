package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp

//Merging, lift, and part of intake done by Ethan, the rest done by Nihal.
public class TwentyTwentyOneOpModeCode extends LinearOpMode {
    private double LeftStickValue;
    private double RightStickValue;
    private Blinker Control_Hub;
    private Blinker expansion_Hub_2;
    ElapsedTime mytimer = new ElapsedTime();
    @Override
    public void runOpMode() {
        InitialLauncherAndIntakeCode.Launcher launcher = new InitialLauncherAndIntakeCode.Launcher();
        InitialLauncherAndIntakeCode.LauncherStates driveOpState = InitialLauncherAndIntakeCode.LauncherStates.Start;
        InitialLifterCode.Lifter lift = new InitialLifterCode.Lifter();
        NihalEthanTest.Launcher Launcher = new NihalEthanTest.Launcher();
        Control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        expansion_Hub_2 = hardwareMap.get(Blinker.class, "Expansion Hub 2");
        lift.LiftMotor = hardwareMap.get(DcMotor.class, "LiftMotor");
        lift.ForkServo = hardwareMap.get(CRServo.class, "LiftServo");
        launcher.LaunchMotor = hardwareMap.get(DcMotor.class, "LaunchMotor");
        launcher.LaunchServo = hardwareMap.get(Servo.class, "LaunchServo");
        waitForStart();

        while (opModeIsActive()) {
            this.LeftStickValue = -gamepad2.left_stick_y;
            this.RightStickValue = -gamepad2.right_stick_y;
            lift.MoveLift(this.LeftStickValue);
            lift.MoveServo(this.RightStickValue);
            telemetry.addData("testing LauncherOn:", launcher.launcherOn);
            telemetry.addData("Lift Power", lift.LiftPower);
            telemetry.addData("Fork Power", lift.ForkPower);
            telemetry.update();
            switch (driveOpState) {
                case Start:
                    if (this.gamepad2.a) {
                        driveOpState = InitialLauncherAndIntakeCode.LauncherStates.ButtonPushed;
                    }

                    if (this.gamepad2.b) {
                        driveOpState = InitialLauncherAndIntakeCode.LauncherStates.Pressed;
                    }
                    if (this.gamepad2.x) {
                        driveOpState = InitialLauncherAndIntakeCode.LauncherStates.ButtonPushed2;
                    }
                    break;
                case Pressed:
                    if (!this.gamepad2.b) {
                        driveOpState = InitialLauncherAndIntakeCode.LauncherStates.firsttimer;

                    }
                    break;
                case firsttimer:
                    mytimer.reset();
                    driveOpState = InitialLauncherAndIntakeCode.LauncherStates.Load;
                    break;

                case Load:
                    launcher.Shoot();
                    if (mytimer.time() >= 0.15) {
                        driveOpState = InitialLauncherAndIntakeCode.LauncherStates.secondtimer;
                    }
                    break;

                case secondtimer:
                    mytimer.reset();
                    driveOpState = InitialLauncherAndIntakeCode.LauncherStates.ResetPosition;
                    break;

                case ResetPosition:
                    launcher.Reload();
                    if (mytimer.time() >= 0.15) {
                        driveOpState = InitialLauncherAndIntakeCode.LauncherStates.Start;
                    }
                    break;

                case ButtonPushed:
                    if (!this.gamepad2.a) {
                        driveOpState = InitialLauncherAndIntakeCode.LauncherStates.ToggleLauncher;
                    }
                    break;
                case ButtonPushed2:
                    if (!this.gamepad2.x) {
                        driveOpState = InitialLauncherAndIntakeCode.LauncherStates.ToggleIntake;
                        break;
                    }

                case ToggleIntake:
                    launcher.IntakeToggle();
                    driveOpState = InitialLauncherAndIntakeCode.LauncherStates.Start;
                    break;
                case ToggleLauncher:
                    launcher.LauncherToggle();
                    driveOpState = InitialLauncherAndIntakeCode.LauncherStates.Start;
                    break;
            }
            launcher.LauncherRun();
        }
    }
}

