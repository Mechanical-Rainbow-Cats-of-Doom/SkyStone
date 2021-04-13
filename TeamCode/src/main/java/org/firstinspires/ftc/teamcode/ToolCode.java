package org.firstinspires.ftc.teamcode;
//Created by Nihal & Ethan
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
public class ToolCode {
    public enum LauncherStates {
        Start,
        ButtonPushed,
        ToggleLauncher,
        Pressed,
        Load,
        ResetPosition,
        firsttimer,
        secondtimer
    }

    public static class Launcher { //The states in the enum
        public DcMotor LaunchMotor;
        public Servo LaunchServo;
        public boolean launcherOn = false;
        private double maxPower = 0.9;

        public void LauncherToggle() {
            launcherOn = !launcherOn;
        }

        public void LauncherRun(double power) {
            if (launcherOn && power <= 1 && power >= -1) {
                power = power * -maxPower;
                LaunchMotor.setPower(power);
            } else {
                LaunchMotor.setPower(0);
            }
        }

        public void Shoot() {
            LaunchServo.setPosition(0.7);
        }

        public void Reload() {
            LaunchServo.setPosition(1.0);
        }
    }

    public static class Grabber {
        public Servo GrabberLeft;
        public Servo GrabberRight;
        private boolean GrabberClosed = false;

        public void Open() {
            GrabberLeft.setPosition(0);
            GrabberRight.setPosition(0);
            GrabberClosed = false;
        }

        public void Close() {
            GrabberLeft.setPosition(1);
            GrabberRight.setPosition(1);
            GrabberClosed = true;
        }

        public void Toggle() {
            if (GrabberClosed) {
                this.Open();
            } else {
                this.Close();
            }
        }
    }

    public static class Lifter {
        public double LiftPower;
        public double ForkPower;
        public DcMotor LiftMotor;
        public CRServo ForkServo;

        public void MoveLift(double LeftStick) {
            if (LeftStick == 0) {
                this.LiftMotor.setPower(-0.1);
                this.LiftPower = -0.1;
            } else if (LeftStick < 0) {
                this.LiftMotor.setPower(LeftStick * -0.46);
                this.LiftPower = (LeftStick * -0.46);
            } else {
                this.LiftMotor.setPower(-LeftStick);
                this.LiftPower = (LeftStick * -1);
            }
        }

        public void MoveServo(double RightStick) {
            this.ForkServo.setPower(RightStick);
            this.ForkPower = RightStick;
        }
    }

    public enum IntakeEnum {
        WaitingForPush,
        WaitingForRelease,
        ChangeValue,
        ChangeMotors,
        WaitingForDpadRelease,
        ChangeFrontValue,
        WaitingForDownRelease,
        SwitchIntakeDirection
    }

    public static class Intake {
        public DcMotor IntakeMotor;
        public DcMotor IntakeMotor2;
        private boolean MotorState1 = false; //false = off, true = on.
        private boolean MotorState2 = false;
        private int intakeDirection = 1;
        ToolCode.IntakeEnum IntakeSwitch = IntakeEnum.WaitingForPush;

        public void Run(boolean x, boolean up, boolean down) {
            switch (IntakeSwitch) {
                case WaitingForPush:
                    if (x) {
                        IntakeSwitch = ToolCode.IntakeEnum.WaitingForRelease;
                    } else if (up) {
                        IntakeSwitch = ToolCode.IntakeEnum.WaitingForDpadRelease;
                    } else if (down) {
                        IntakeSwitch = ToolCode.IntakeEnum.WaitingForDownRelease;
                    } else {
                        IntakeSwitch = ToolCode.IntakeEnum.ChangeMotors;
                    }
                    break;
                case WaitingForRelease:
                    if (!x) {
                        IntakeSwitch = ToolCode.IntakeEnum.ChangeValue;
                    }
                    break;
                case ChangeValue:
                    MotorState2 = !MotorState2;
                    MotorState1 = !MotorState1;
                    IntakeSwitch = ToolCode.IntakeEnum.ChangeMotors;
                    break;
                //56.5 13 4
                case WaitingForDpadRelease:
                    if (!up) {
                        IntakeSwitch = ToolCode.IntakeEnum.ChangeFrontValue;
                    }
                    break;
                case ChangeFrontValue:
                    MotorState1 = !MotorState1;
                    IntakeSwitch = ToolCode.IntakeEnum.ChangeMotors;
                    break;
                case WaitingForDownRelease:
                    if (!up) {
                        IntakeSwitch = ToolCode.IntakeEnum.SwitchIntakeDirection;
                    }
                    break;
                case SwitchIntakeDirection:
                    intakeDirection *= -1;
                    IntakeSwitch = ToolCode.IntakeEnum.ChangeMotors;
                    break;
                case ChangeMotors:
                    if (MotorState2) {
                        IntakeMotor2.setPower(1);
                    } else if (!MotorState2) {
                        IntakeMotor2.setPower(0);
                    }
                    if (MotorState1) {
                        IntakeMotor.setPower(-1 * intakeDirection);
                    } else if (!MotorState1) {
                        IntakeMotor.setPower(0);
                    }
                    IntakeSwitch = ToolCode.IntakeEnum.WaitingForPush;
                    break;
            }
        }
    }

    public enum RingWiper {
        WaitingForPushY,
        WaitingForReleaseY,
        ToggleValue,
        ChangeServo
    }

    public static class Wiper {
        ToolCode.RingWiper RingWiperSwitch = RingWiper.WaitingForPushY;
        public Servo WiperServo;
        private boolean servoState = false;
        public void Run(boolean y) {
            switch (RingWiperSwitch) {
                case WaitingForPushY:
                    if (y) {
                        RingWiperSwitch = ToolCode.RingWiper.WaitingForReleaseY;
                    } else {
                        RingWiperSwitch = ToolCode.RingWiper.ChangeServo;
                    }
                    break;
                case WaitingForReleaseY:
                    if (!y) {
                        RingWiperSwitch = ToolCode.RingWiper.ToggleValue;
                    }
                    break;
                case ToggleValue:
                    servoState = !servoState;
                    RingWiperSwitch = ToolCode.RingWiper.ChangeServo;
                    break;
                case ChangeServo:
                    if (servoState) {
                        WiperServo.setPosition(0.75);
                    } else if (!servoState) {
                        WiperServo.setPosition(.95);
                    }
                    RingWiperSwitch = ToolCode.RingWiper.WaitingForPushY;
                    break;
            }
        }
    }
}



