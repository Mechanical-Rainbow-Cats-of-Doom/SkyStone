/*
Copyright 2019 FIRST Tech Challenge Team 17235

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

PURPOSE:
This code is example code, meant to show a few different concepts.  The code configures the
expansion hub resources in a way that matches the Mr COD robot from 2019.
FUNCTION:
1. Sets up a TeleOp mode that allows for the control of the robot's motion
2. Sets up a state machine that supports basic motion as well as example code for creating driver
      assist functions
3. Shows some of the Java basics, like creating specific classes for different subsystems
*/
package org.firstinspires.ftc.teamcode;

/*
This section defines all of the Java packages required for using the different FTC resources
NOTE: This is something you will need for every OpMode you create.  It is also something that
can be put together into a separate package/file that you can just include in every block of
code you write.
 */
import com.qualcomm.hardware.bosch.BNO055IMU;  //This is the package for controlling the IMU
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.lang.Math;  //This is the standard Java package for a variety of math functions
import java.math.BigDecimal;

/**
 * This file contains a minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode allows you to have basic control of your robot during Driver Mode.
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@TeleOp

public class EthanMTestNotNihalNotPoggersTestas extends LinearOpMode {
    /*
    This section maps the resources that you defined in the FTC Controller configuration.
    These are variable declarations, where you are assigning the resource to the corresponding
    Java package for the resource.
    NOTE: This is something you will need for every OpMode you create.  It is also something that
    can be put together into a separate package/file that you can just include in every block of
    code you write.  This section should probably be standardized for your all of your projects
    since you all are using the same robot.
    Note:  The name of this class will be what shows up on your controllers.
     */
    //Declaration of the two expansion hubs
    private Blinker expansion_Hub_1;
    private DcMotor BigMotor;
    private CRServo ServoPower;
    private double LeftStickValue;
    //Declaration for the sensors.  These classes were created to work with the default/supported
    //sensors on the robot.  Technically, if we ever run into a sensor that has the same interface
    //as one of the ports on the Expansion Hub, you could design your own interface.

    //Declaration for the Inertial Measurement Unit (IMU)--note:  This is the built in gyroscope
    //in the Expansion Hub.

    //Below is the declaration for the digital "red/blue" switch we have on our old robot.
    //The system has a collection of other interfaces that can be used regularly.
    //NOTE:  For those of you using motion control/needing start/stop locations, you could use
    //something like this for a limit switch or something--and then use it as part of a closed loop
    //control algorithm.


    /*
    CLASSES
    Note:  The next section declares a couple of classes.  For more information on the parts of a
    class, please refer to https://www.developer.com/java/ent/article.php/3488176/The-Components-of-a-Class.htm
    Note2: You will probably see that I've made some mistakes with my use of "public" and "private".
    You can get a really good description of correct practice at:
    https://therenegadecoder.com/code/the-difference-between-private-and-public-in-java/
    If you find mistakes in what I've done, CHANGE THEM!  You can also take a pass at calling them
    out publicly--nothing like a little shame to ensure the coach is doing things correctly.
    (If you do make changes, though, test them--nothing screws up your flexing like checking in bad
    code!)
     */

    /*
    Class: Chassis
    Attributes:
        frontLeft: variable representing the desired/current speed of the front left motor
        backLeft:  variable representing the desired/current speed of the back left motor
        frontRight: variable representing the desired/current speed of the front right motor
        backRight: variable representing the desired/current speed of the back right motor
     Accessors:
        SetMotors: This function takes arguments drive, strafe, rotate; translates them into
            values the motors will recognize; and saves them in the corresponding attributes.
        Drive:  This function takes the values of the attributes and sets the motor power.
     */



    /*
    Class: UberTool
    Attributes: None
    Accessors:
        Lift: This sets the power of the scissors lift motor to the value of the argument liftPower
        Clamp: This sets the power of the clamp motor to the value of the argument clampPower
     */


    /*
    OperState is a enumerated type that is used to define the states in the main OpMode.
    Below, the variable driveOpState will be declared to be of this time and the states will be
    interpreted as follows:
    NORMALDRIVE:  When in this state, the software will give the controller full ability to move
        everything from the chassis motors to the lift/clamp.  This is where the "default" behavior
        of the system is supported.  Based on certain controller actions, the system can be moved
        to other states.
    ROTATEPOSITION: When in this state, the software will "automatically" control the robot,
        rotating it until the robot is facing a specific angle relative to the starting position.

     Due to the way Java handles enum classes, I was not able to get this type to be recognized
     anywhere else in the code--I'm still learning why, but it appears to be due to some gaps in
     the way Java supports inheritance of enums.
     Activity for the reader:  teach the team why this is!
     */

    /*
    This is the main accessor (function) for the whole OpMode.
    You will write most of your custom code here.
     */
    @Override
    public void runOpMode() {
        /*
        INITIALIZATION SECTION
        The initialization section in this code doesn't do much--just configs the robot resources
        and initializes the IMU.
         */
        /*
        ROBOT RESOURCE ACCESS

        For now, let's call these "magic functions".  I'm not sure exactly what's happening in these,
        but they are required for accessing the FTC resources.
        The names in the deviceName argument must EXACTLY match those in the configuration on your
        controller.
         */
        expansion_Hub_1 = hardwareMap.get(Blinker.class, "Nihal");
        BigMotor = hardwareMap.get(DcMotor.class, "Big Motor");
        ServoPower = hardwareMap.get(CRServo.class,"ServoPower");

        /*
        IMU CONFIGURATION
         */
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.

        //parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".


        // Send telemetry message to alert driver that we are calibrating;

        /*
        END OF INITIALIZATION SECTION
         */
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        /*
        MAIN SECTION FOR MATCH
         */
        // run until the end of the match (driver presses STOP)
        /*
        Variable declarations that are used by software.
        Note:  I left out public/private/static declarations--what would be appropriate here?
         */
        /*
        Note: In this example, I don't do any other "first time" operations on the software--the
        code immediately drops into the while loop.  If you wanted to do some additional post-PLAY
        initialization, you would do it here before going to the while loop.
         */

        /*
        MAIN WHILE LOOP
        This is the main loop where all of the work is done.
        ONE KEY CONSIDERATION:
        This loop will be traversed continuously--it represents a slice of time of the robot's overall
        performance.  Keep in mind that we want this loop to take as close to 0 time as possible.
        Running most code will take nearly 0 time.  Each sensor you read will take between 3-7 ms.
        Again, that's not much unless you start reading everything every time you go through a loop.
        The faster this loop is, the more responsive your code will be.
        If you start seeing slow response from your robot when you're using it, it's probably
        because it takes too much time to process the code in this loop.
        ***This concern is a major reason for using a state machine in the loop--that way, you're
        only ever looking at the stuff you really care about.
         */
        while (opModeIsActive())
            telemetry.addData("left stick y axis", this.gamepad1.left_stick_y);
            telemetry.addData("right stick y axis", this.gamepad1.right_stick_y);
            telemetry.update();
            LeftStickValue = this.gamepad1.left_stick_y;
            if (LeftStickValue == 0) {
                BigMotor.setPower(0.001);
            }
            else {
                BigMotor.setPower(LeftStickValue);
            }
            ServoPower.setPower(this.gamepad1.right_stick_y);



            /*
            void set
            UNIVERSAL ACTIONS
            These actions will happen every time you go through the while loop.
            As noted above, keep in mind that we'll want to keep these to a minimum.
             */
            /*
            IMU angle reading
            There are more things that can be done than just reading the angles--and I only use the
            Z angle for this example.
            The IMU is actually pretty powerful--you can also measure acceleration and some other
            things.  As an example, you might be able to use the IMU to detect when you hit a wall
            at full speed.
             */




        }
    }
