package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp

public class InitialLifterCode extends LinearOpMode {
    private Blinker expansion_Hub_1;
    private double LeftStickValue;
    private double RightStickValue;

    public static class Lifter {
        public double LiftPower;
        public double ForkPower;
        public DcMotor LiftMotor;
        public CRServo ServoPower;
        public void MoveLift(double LeftStick) {
            if (LeftStick == 0) {
                this.LiftMotor.setPower(-0.215);
                this.LiftPower = -0.215;
            }

            if (LeftStick < 0) {
                   this.LiftMotor.setPower(LeftStick * -0.46);
                    this.LiftPower = (LeftStick * -0.46);
                } else {
                    this.LiftMotor.setPower(-LeftStick);
                    this.LiftPower = (LeftStick * -1);
                }
            }

        public void MoveServo(double RightStick) {
            this.ServoPower.setPower(RightStick);
            this.ForkPower = RightStick;
        }
    }

    @Override
    public void runOpMode() {
        InitialLifterCode.Lifter lift = new InitialLifterCode.Lifter();
        expansion_Hub_1 = hardwareMap.get(Blinker.class, "Nihal");
        lift.LiftMotor = hardwareMap.get(DcMotor.class, "LiftMotor");
        lift.ServoPower = hardwareMap.get(CRServo.class, "LiftServo");
        waitForStart();
        while (opModeIsActive()) {
            this.LeftStickValue = -gamepad2.left_stick_y;
            this.RightStickValue = -gamepad2.right_stick_y;
            lift.MoveLift(this.LeftStickValue);
            lift.MoveServo(this.RightStickValue);
            telemetry.addData("Lift Power", lift.LiftPower);
            telemetry.addData("Fork Power", lift.ForkPower);
            telemetry.update();
        }
    }
}
