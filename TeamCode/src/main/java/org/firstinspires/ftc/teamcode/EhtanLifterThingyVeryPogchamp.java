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
    private double RightStickValue;

    private class Lifter {
        private void MoveLift(double LeftStick) {
            if (LeftStick == 0) { // Do not complain about my use of multiple if statements or you will perish
                LiftMotor.setPower(1);
            }

            if (LeftStick < 0) {
                    LiftMotor.setPower(LeftStick * 0.55);
                } else {
                    LiftMotor.setPower(LeftStick);
                }
            }

        public void MoveServo(double RightStick) {
            ServoPower.setPower(RightStick);
        }
    }


    @Override
    public void runOpMode() {
        EhtanLifterThingyVeryPogchamp.Lifter lift = new EhtanLifterThingyVeryPogchamp.Lifter();

        expansion_Hub_1 = hardwareMap.get(Blinker.class, "Nihal");
        LiftMotor = hardwareMap.get(DcMotor.class, "Big Motor");
        ServoPower = hardwareMap.get(CRServo.class, "ServoPower");
        waitForStart();
        while (opModeIsActive()) {
            LeftStickValue = -gamepad1.left_stick_y;
            RightStickValue = -gamepad1.right_stick_y;
            telemetry.addData("left stick y axis", LeftStickValue);
            telemetry.addData("right stick y axis", RightStickValue);
            telemetry.update();
            lift.MoveLift(LeftStickValue);
            lift.MoveServo(RightStickValue);
        }


    }
}
