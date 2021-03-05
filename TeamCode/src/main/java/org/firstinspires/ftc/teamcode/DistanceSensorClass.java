package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
public class DistanceSensorClass {
    public static class RingClass {

        public DistanceSensor DistanceSensor;
        double AveragedArray;
        double total;
        int index = 0;
        int ArraySize = 50;
        double[] sensorArray = new double [ArraySize];
        int RingCount = 0;

        public void MeasureDistance() {
            if (index >= (ArraySize - 1)) {
                index = 0;
            }
            else {
                index++;
            }
            sensorArray[index] = DistanceSensor.getDistance(DistanceUnit.INCH);
            total = 0;
            for (int i = 0; i <= (ArraySize - 1); i++) {
                total = total + sensorArray[i];
            }
            AveragedArray = total / sensorArray.length;

            if (AveragedArray <= 17.6) {
                RingCount = 4;
            }
            else if (AveragedArray <= 19) {
                RingCount = 1;
            }
            else if (AveragedArray <= 19.7) {
                RingCount = 0;
            }
        }

        public int RingHeight() {
            return (RingCount);
        }

    }
}
