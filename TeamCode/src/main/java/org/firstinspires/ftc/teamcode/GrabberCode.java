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
           //GrabberRight.setPosition(0);
            GrabberClosed = false;
        }
        private void Close() {
            GrabberLeft.setPosition(0.5);
            //GrabberRight.setPosition(1);
            GrabberClosed = true;
        }
        public void Toggle(boolean rbump) {
            if (rbump && !Waiting) { Waiting = true; }
            else if (Waiting && !rbump) {
                Waiting = false;
                if (GrabberClosed) { this.Open(); }
                else { this.Close();}
            }
        }
    }
}
