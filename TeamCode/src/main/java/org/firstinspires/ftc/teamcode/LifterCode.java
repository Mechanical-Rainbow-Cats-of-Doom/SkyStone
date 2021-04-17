package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
//Created by Ethan Sanders
public class LifterCode {
    public static class Lifter {
        public double LiftPower;
        public double ForkPower;
        public DcMotor LiftMotor;
        public void MoveLift(double Power) {
            if (Power == 0) {
                this.LiftMotor.setPower(-0.1);
                this.LiftPower = -0.1;
            } else if (Power < 0) {
                this.LiftMotor.setPower(Power * -0.46);
                this.LiftPower = (Power * -0.46);
            } else {
                this.LiftMotor.setPower(-Power);
                this.LiftPower = (Power * -1);
            }
        }

    }
}
