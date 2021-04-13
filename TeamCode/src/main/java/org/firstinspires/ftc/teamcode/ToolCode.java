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
    public static class Launcher {
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
            if (GrabberClosed) { this.Open(); }
            else { this.Close(); }
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
}



