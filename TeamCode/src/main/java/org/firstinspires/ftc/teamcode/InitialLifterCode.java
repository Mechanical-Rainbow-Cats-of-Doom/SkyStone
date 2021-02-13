package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp

public class InitialLifterCode extends LinearOpMode {
    private Blinker expansion_Hub_1;
    private DcMotor LiftMotor;
    private CRServo ServoPower;
    private double LeftStickValue;
    private double RightStickValue;

    private class Lifter {
        private double LiftPower;
        private double ForkPower;
        private void MoveLift(double LeftStick) {
            if (LeftStick == 0) {
                LiftMotor.setPower(-0.1);
                this.LiftPower = -0.1;
            }

            if (LeftStick < 0) {
                    LiftMotor.setPower(LeftStick * -0.46);
                    this.LiftPower = (LeftStick * -0.46);
                } else {
                    LiftMotor.setPower(LeftStick * -0.8);
                    this.LiftPower = (LeftStick * -0.8);
                }
            }

        public void MoveServo(double RightStick) {
            ServoPower.setPower(RightStick);
            this.ForkPower = RightStick;
        }
    }

    @Override
    public void runOpMode() {
        InitialLifterCode.Lifter lift = new InitialLifterCode.Lifter();
        expansion_Hub_1 = hardwareMap.get(Blinker.class, "Nihal");
        LiftMotor = hardwareMap.get(DcMotor.class, "Big Motor");
        ServoPower = hardwareMap.get(CRServo.class, "ServoPower");
        waitForStart();
        while (opModeIsActive()) {
            LeftStickValue = -gamepad1.left_stick_y;
            RightStickValue = -gamepad1.right_stick_y;
            lift.MoveLift(LeftStickValue);
            lift.MoveServo(RightStickValue);
            telemetry.addData("Lift Power", lift.LiftPower);
            telemetry.addData("Fork Power", lift.ForkPower);
            telemetry.update();
        }
    }
}
