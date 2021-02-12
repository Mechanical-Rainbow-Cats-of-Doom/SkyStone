package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp

public class EhtanLifterThingyVeryPogchamp extends LinearOpMode {
    private Blinker expansion_Hub_1;
    private DcMotor LiftMotor;
    private CRServo ServoPower;
    private double LeftStickValue;
    private class Lifter {
        double LeftStickValue = gamepad1.left_stick_y;
        double RightStickValue = gamepad1.right_stick_y;

        private void MoveLift () {
            if (LeftStickValue == 0) { // Do not complain about my use of multiple if statements or you will perish
                LiftMotor.setPower(0.09);
            } else {
                if (LeftStickValue < 0) {
                    LiftMotor.setPower(LeftStickValue * 0.55);
                } else {
                    LiftMotor.setPower(LeftStickValue);
                }
            }
        }
        private void MoveServo () {
            ServoPower.setPower(RightStickValue);
        }
    }


    @Override
        public void runOpMode () {
            EhtanLifterThingyVeryPogchamp.Lifter lift = new EhtanLifterThingyVeryPogchamp.Lifter();
            lift.MoveLift();
            lift.MoveServo();
    }


    }

