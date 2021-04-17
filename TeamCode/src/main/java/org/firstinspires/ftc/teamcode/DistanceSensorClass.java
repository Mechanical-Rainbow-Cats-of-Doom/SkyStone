package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
public class DistanceSensorClass {
    /* All of this code is needed as there is a lot of noise from the distance sensor.
    This code makes sure that the output from the distance sensor is reliable and isn't completely off from reality.
     */
    public static class RingClass {

        public DistanceSensor DistanceSensor;
        public double AveragedArray;
        private double total;
        private int index = 0;
        private int ArraySize = 41; //Allows for easy way to change the size of the array that affects all of the code
        private double[] sensorArray = new double [ArraySize];
        private int RingCount = 0;

        public void MeasureDistance() {
            //Sets the index for the array, sets to 0 if full
            if (index >= (ArraySize - 1)) {
                index = 0;
            }
            else {
                index++;
            }
            sensorArray[index] = DistanceSensor.getDistance(DistanceUnit.INCH); //Puts distance sensor to the current value of the index

            total = 0; //Resetting Variable from all pasts runs of code

            //This loop adds up all variable in the array into a single value
            for (int i = 0; i <= (ArraySize - 1); i++) {
                total = total + sensorArray[i];
            }
            AveragedArray = total / sensorArray.length; //Averages the value spit out of the loop

            //This loop uses measured values from the field to determine the amount of rings based off of the averaed valu.
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

        //This is the output of all of this code, and allows the amount of rings to be used in autonomous.
        public int RingHeight() {
            return (RingCount);
        }

    }
}
