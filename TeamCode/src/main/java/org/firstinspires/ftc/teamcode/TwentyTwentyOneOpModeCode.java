package org.firstinspires.ftc.teamcode;

import android.content.ContentResolver;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class TwentyTwentyOneOpModeCode extends LinearOpMode {
    private double LeftStickValue;
    private double RightStickValue;
    private Blinker Control_Hub;
    private Blinker expansion_Hub_2;

    @Override

    public void runOpMode() {
        InitialLifterCode.Lifter lift = new InitialLifterCode.Lifter();
        NihalEthanTest.Launcher Launcher = new NihalEthanTest.Launcher();
        Control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        expansion_Hub_2 = hardwareMap.get(Blinker.class, "Expansion Hub 2");
        lift.LiftMotor = hardwareMap.get(DcMotor.class, "LiftMotor");
        lift.ForkServo = hardwareMap.get(CRServo.class, "LiftServo");
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
