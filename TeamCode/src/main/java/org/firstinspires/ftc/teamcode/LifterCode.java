package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
//Created by Ethan Sanders
public class LifterCode {
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
