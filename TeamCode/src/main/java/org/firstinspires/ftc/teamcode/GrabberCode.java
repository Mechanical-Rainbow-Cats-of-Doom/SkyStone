package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.Servo;
public class GrabberCode {
    public static class Grabber {
        public Servo GrabberLeft;
        public Servo GrabberRight;
        private boolean GrabberClosed = false;
        private boolean Waiting = false;
        private void Open() {
            GrabberLeft.setPosition(0);
           // GrabberRight.setPosition(0);
            GrabberClosed = false;
        }
        private void Close() {
            GrabberLeft.setPosition(1);
            GrabberRight.setPosition(1);
            GrabberClosed = true;
        }

        public void Toggle(boolean x) {
            if (x && !Waiting) { Waiting = true; }
            else if (Waiting && !x) {
                Waiting = false;
                if (GrabberClosed) { this.Open(); }
                else if (!GrabberClosed) { this.Close();}
            }
        }
    }
}
