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
    private double LiftPower;
    private double ForkPower;


    private class Lifter {
        private void MoveLift(double LeftStick, double InLiftPower) {
            if (LeftStick == 0) {
                LiftMotor.setPower(0.07);
                InLiftPower = 0.07;
            }

            if (LeftStick < 0) {
                    LiftMotor.setPower(LeftStick * 0.46);
                    InLiftPower = LeftStick * 0.46;
                } else {
                    LiftMotor.setPower(LeftStick);
                    InLiftPower = LeftStick;
                }
            }

        public void MoveServo(double RightStick, double InForkPower) {
            ServoPower.setPower(RightStick);
            InForkPower = RightStick;
        }
    }


    @Override
    public void runOpMode() {
        InitialLifterCode.Lifter lift = new InitialLifterCode.Lifter();
        expansion_Hub_1 = hardwareMap.get(Blinker.class, "Nihal");
        LiftMotor = hardwareMap.get(DcMotor.class, "Big Motor");
        ServoPower = hardwareMap.get(CRServo.class, "ServoPower");
        telemetry.addData("Lift Power", LiftPower);
        telemetry.addData("Fork Power", ForkPower);
        waitForStart();
        while (opModeIsActive()) {
            LeftStickValue = -gamepad1.left_stick_y;
            RightStickValue = -gamepad1.right_stick_y;
            telemetry.update();
            lift.MoveLift(LeftStickValue, LiftPower);
            lift.MoveServo(RightStickValue, ForkPower);
        }


    }
}
