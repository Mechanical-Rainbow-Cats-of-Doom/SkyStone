package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
//Created by Ethan Sanders
public class InitialLifterCode {
    private Blinker expansion_Hub_1;
    private double LeftStickValue;
    private double RightStickValue;

    public static class Lifter {
        public double LiftPower;
        public double ForkPower;
        public DcMotor LiftMotor;
        public CRServo ForkServo;

        public double MoveLift(double LeftStick) {
            double banana = 0;
            if (LeftStick == 0) {
                this.LiftMotor.setPower(-0.3);
                this.LiftPower = -0.3;
                banana = 1;
            }

            if (LeftStick < 0) {
                this.LiftMotor.setPower(LeftStick * -0.46);
                this.LiftPower = (LeftStick * -0.46);
                banana = 2;
            } else {
                this.LiftMotor.setPower(-LeftStick);
                this.LiftPower = (LeftStick * -1);
                banana = 3;
            }
            return banana;
        }

        public void MoveServo(double RightStick) {
            this.ForkServo.setPower(RightStick);
            this.ForkPower = RightStick;
        }
    }
}
