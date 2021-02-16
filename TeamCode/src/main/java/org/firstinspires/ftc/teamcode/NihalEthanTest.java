/*
Copyright 2019 FIRST Tech Challenge Team 17235

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

PURPOSE:
This code is example code, meant to show a few different concepts.  The code configures the
expansion hub resources in a way that matches the Mr COD robot from 2019.
FUNCTION:
1. Sets up a TeleOp mode that allows for the control of the robot's motion
2. Sets up a state machine that supports basic motion as well as example code for creating driver
      assist functions
3. Shows some of the Java basics, like creating specific classes for different subsystems
*/
package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp

public class NihalEthanTest extends LinearOpMode {

    private Blinker expansion_Hub_1;
    ElapsedTime mytimer = new ElapsedTime();

    public static class Launcher {
        boolean launcherOn = false;
        public DcMotor LaunchMotor;
        public Servo LaunchServo;
        public void LauncherToggle () {
            launcherOn = !launcherOn;
        }

        public void LauncherRun () {
            if (launcherOn) {
                LaunchMotor.setPower(-1);
            }
            else {
                LaunchMotor.setPower(0);
            }
        }

        public void Shoot () {
            LaunchServo.setPosition(0.8);
        }

        public void Reload () {
            LaunchServo.setPosition(1.0);
        }

    }

    enum OperState {
        Start,
        ButtonPushed,
        ToggleLauncher,
        Pressed,
        Load,
        ResetPosition,
        firsttimer,
        secondtimer
    }

    @Override
    public void runOpMode() {
        NihalEthanTest.Launcher NihalLauncher = new NihalEthanTest.Launcher();

        expansion_Hub_1 = hardwareMap.get(Blinker.class, "Nihal");
        NihalLauncher.LaunchMotor = hardwareMap.get(DcMotor.class, "LaunchMotor");
        NihalLauncher.LaunchServo = hardwareMap.get(Servo.class,"LaunchServo");
        //boolean LauncherOn = false;

        waitForStart();
        telemetry.addData("testing A button:", this.gamepad1.a);
        telemetry.addData("testing LauncherOn:", NihalLauncher.launcherOn);

        telemetry.update();


        OperState driveOpState = OperState.Start;
        telemetry.addData("State", driveOpState);

        while (opModeIsActive()) {
            switch (driveOpState) {
                case Start:
                    if (this.gamepad1.a) {
                        driveOpState = OperState.ButtonPushed;
                    }

                    if (this.gamepad1.b) {
                        driveOpState = OperState.Pressed;
                    }
                    break;
                case Pressed:
                    if (!this.gamepad1.b) {
                        driveOpState = OperState.firsttimer;

                    }
                    break;
                case firsttimer:
                    mytimer.reset();
                    driveOpState = OperState.Load;
                    break;

                case Load:
                    NihalLauncher.Shoot();
                    if (mytimer.time() >= 0.15){
                        driveOpState = OperState.secondtimer;
                    }
                    break;

                case secondtimer:
                    mytimer.reset();
                    driveOpState = OperState.ResetPosition;
                    break;

                case ResetPosition:
                    NihalLauncher.Reload();
                    if (mytimer.time() >= 0.15) {
                        driveOpState = OperState.Start;
                    }
                    break;

                case ButtonPushed:
                    if (!this.gamepad1.a) {
                        driveOpState = OperState.ToggleLauncher;
                    }
                    break;

                case ToggleLauncher:
                    NihalLauncher.LauncherToggle();
                    driveOpState = OperState.Start;
                    break;
                    }
            }
            telemetry.addData("State", driveOpState);
            telemetry.addData("testing A button:", this.gamepad1.a);
            telemetry.addData("testing LauncherOn:", NihalLauncher.launcherOn);
            telemetry.addData("testing B button:", this.gamepad1.b);
            telemetry.update();
        }

    }


