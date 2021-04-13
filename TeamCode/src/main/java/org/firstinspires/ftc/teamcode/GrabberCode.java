package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.Servo;
public class GrabberCode {
    public static class Grabber {
        public Servo GrabberLeft;
        public Servo GrabberRight;
        private boolean GrabberClosed = false;

        public void Open() {
            GrabberLeft.setPosition(0);
            GrabberRight.setPosition(0);
            GrabberClosed = false;
        }
        public void Close() {
            GrabberLeft.setPosition(1);
            GrabberRight.setPosition(1);
            GrabberClosed = true;
        }
        public void Toggle() {
            if (GrabberClosed) { this.Open(); }
            else { this.Close(); }
        }
    }
}
