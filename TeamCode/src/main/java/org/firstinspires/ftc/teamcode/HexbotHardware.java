/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

// Use HexbotHardware class for hardware functions that are not game/season specific
public class HexbotHardware {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private DcMotorEx alphaDrive = null;
    private DcMotorEx gammaDrive = null;
    private DcMotorEx betaDrive = null;
    private DcMotorEx arm = null;
    // Declare OpMode members for servos.
    private CRServo endeffector = null;
    private ServoImplEx turntable = null;
    private Servo grabber_A = null;
    private Servo grabber_B = null;
    // Declare Sensor objects
    private IMU imu = null;
    private AnalogInput endeffectorangle = null;
    // Declare constants for motion limitation
    private final double MAX_DRIVE_POWER = 0.5d;
    // Doubled up springs on the 4-bar linkage, so hopefully mechanism is more balanced up/down.
    // Note, with one spring per side, constants were 1.0 up and 0.1 down.
    private static final double ARM_MAX_UP_POWER    =  0.5 ;
    private static final double ARM_MAX_DOWN_POWER  = 0.4 ;


    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    // Assuming for now that we want servo midpoint and move speed to be same for all servos
    public static final double MID_SERVO       =  0.5 ;
    public static final double SERVO_SPEED      =  0.02 ;  // sets rate to move turntable servo

    // Using classic AndyMark NeverRest 4o meters for drive wheels
    // 28 ticks per rotation for motor only, times GR 40 = 1120 ticks/rotation
    private static final int DRIVE_TICKS_PER_ROT = 1120;
    // Using Tetrix Max 4 in Omni Wheels
    private static final double WHEEL_DIAM = 4.0d;

    // Define fields for saving information about position and speed between iterations of main loop
    private double[] position = {0,0};      // x and y positions in inches
    private double[] speed = {0,0};         // x and y positions in inches/second
    private int[] wheel_travel = {0,0,0};   // alpha, beta, and gamma encoder counts
    private double linear_distance = 0;     // estimated total linear distance in inches
    private static double VOLTS_PER_DEGREE = 0.025; // just a guess todo: measure volts per degree for ee
    private LookupTable v_level;
    // Create PID controllers for robot turning and robot distance
    private Control turn_control = new Control(1,0,0); // todo: tune PID for turn control
    private Control dist_control = new Control(1,0,0); // todo: tune PID for distance control
    private double target_heading = 0;
    private double max_speed = 10.0;


    // Define a constructor that allows the OpMode to pass a reference to itself.
    public HexbotHardware(LinearOpMode opmode) {
        this(opmode,0d,0d);
    }
    // Overload constructor to allow opmode to pass in a starting position
    public HexbotHardware(LinearOpMode opmode, double start_x,double start_y) {
        myOpMode = opmode;
        position[0] = start_x;
        position[1] = start_y;

        // Create a lookup table with:
        //      Key = Integer (encoder counts for arm motor)
        //      Value = Double (voltage on the potentiometer for the "level" zero position of end effector)
        v_level = new LookupTable();
        v_level.addRow(0,0.0d); // build lookuptable here todo: measure voltage at various arm positions
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init()    {
        // Define and Initialize Motors, Servos, Sensors
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        alphaDrive  = myOpMode.hardwareMap.get(DcMotorEx.class, "drive_3");
        gammaDrive  = myOpMode.hardwareMap.get(DcMotorEx.class, "drive_2");
        betaDrive = myOpMode.hardwareMap.get(DcMotorEx.class, "drive_1");
        arm = myOpMode.hardwareMap.get(DcMotorEx.class, "arm_0");
        imu = myOpMode.hardwareMap.get(IMU.class, "imu");
        endeffector = myOpMode.hardwareMap.get(CRServo.class,"ee_0");
        endeffectorangle = myOpMode.hardwareMap.get(AnalogInput.class,"pot_0");
        turntable = myOpMode.hardwareMap.get(ServoImplEx.class,"tt_1");
        grabber_A = myOpMode.hardwareMap.get(Servo.class,"grabber_2");
        grabber_B = myOpMode.hardwareMap.get(Servo.class,"grabber_3");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        alphaDrive.setDirection(DcMotor.Direction.REVERSE);
        gammaDrive.setDirection(DcMotor.Direction.REVERSE);
        betaDrive.setDirection(DcMotor.Direction.REVERSE);
        arm.setDirection(DcMotor.Direction.REVERSE);

        // Expand PWM Range to maximum supported by HS-485HB to extend rotation range to 200 degrees
        turntable.setPwmRange(new PwmControl.PwmRange(553,2425));

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        alphaDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        gammaDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        betaDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // initialize wheel_travel to initial encoder values from motors
        wheel_travel[0] = alphaDrive.getCurrentPosition();
        wheel_travel[1] = betaDrive.getCurrentPosition();
        wheel_travel[2] = gammaDrive.getCurrentPosition();

        // Define Hub orientation
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    /*
        *****
        * DRIVEBASE METHODS
        *****
     */

    /**
     * Gets current heading of the robot
     * No need to do this if already using xydrive method which returns heading
     *
     * @return          Current heading in degrees
     * @see #xydrive(double, double, double) drive method already returns current heading
     */
    public double getHeading() {
        // Get robot current heading
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        // return current heading
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    /**
     * Calculates the motor powers required to achieve the requested
     * robot motions: Drive (Axial motion) and Turn (Yaw motion).
     * Then sends these power levels to the motors.
     *
     * @param drive_x   X direction driving power (-1.0 to 1.0) +ve is forward
     * @param drive_y   Y direction driving power
     * @param turn      Right/Left turning power (-1.0 to 1.0) +ve is CW
     * @return          Current heading in degrees
     */
    public double xydrive(double drive_x, double drive_y, double turn) {
        /*
         We assume a three omniwheel holonomic robot with wheels 120 degrees offset, named
         alpha, beta, and gamma, such that "front" of robot is at point between alpha and beta like so:
              front of robot
                    /\
            alpha  /  \ beta
                  /    \
                 /______\
                   gamma
        So measuring from front of robot:
            alpha is at -60 degrees (or -PI/3 radians)
            beta is at +60 degrees (or +PI/3 radians)
            gamma is at 180 degrees (or PI radians)

        We can achieve a speed v in each linear direction and yaw rate by setting power F at each wheel
        In matrix terms, this givex [V] = [X] * [F]
        Where X (x, y, yaw) is
        [sin(alpha) sin(beta)   sin(gamma)  ]
        [cos(alpha) cos(beta)   cos(gamma)  ]
        [1          1           1           ]

        We can calculate power at each wheel based on a desired speed by inverting matrix X:
        [F] = [X}^-1 * [A]
         */
        // Get robot current heading
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        // Calculate sin and cos of angles of each motor alpha, beta, and gamma:
        double[] angles = {orientation.getYaw(AngleUnit.RADIANS)-(Math.PI/3), orientation.getYaw(AngleUnit.RADIANS)+(Math.PI/3), orientation.getYaw(AngleUnit.RADIANS)+Math.PI};
        double[] sin = {Math.sin(angles[0]),Math.sin(angles[1]),Math.sin(angles[2])};
        double[] cos = {Math.cos(angles[0]),Math.cos(angles[1]),Math.cos(angles[2])};
        // Calculate the determinant of our X matrix, [X]^-1 in description above:
        double determinant = sin[0]*(cos[1]-cos[2])+sin[1]*(cos[2]-cos[0])+sin[2]*(cos[0]-cos[1]);
        // Lastly we can determine the desired drive power for each wheel:
        double F_alpha = (drive_x*(cos[1]-cos[2])+drive_y*(sin[2]-sin[1])+turn*(sin[1]*cos[2]-sin[2]*cos[1]))/determinant;
        double F_beta = (drive_x*(cos[2]-cos[0])+drive_y*(sin[0]-sin[2])+turn*(sin[2]*cos[0]-sin[0]*cos[2]))/determinant;
        double F_gamma = (drive_x*(cos[0]-cos[1])+drive_y*(sin[1]-sin[0])+turn*(sin[0]*cos[1]-sin[1]*cos[0]))/determinant;

        // Scale the values so none exceed maximum drive power
        double max = Math.max(Math.max(Math.abs(F_alpha), Math.abs(F_beta)), Math.abs(F_gamma));
        if (max > MAX_DRIVE_POWER) {
            F_alpha /= max;
            F_beta /= max;
            F_gamma /= max;
        }

        // Use existing function to drive three wheels.
        setDrivePower(F_alpha,F_beta,F_gamma);
        // Update position parameters based on the heading pulled from the IMU above
        // We pass the sin and cos arrays to avoid pulling heading again or repeating the math
        updatePosition(sin,cos);
        // return current heading
        return orientation.getYaw(AngleUnit.DEGREES); // todo: consider returning position and heading
    }

    /**
     * Calculates the motor powers required to achieve the requested
     * robot motions: Drive (Axial motion) and Turn (Yaw motion).
     * This version uses PID control to create a turn command to control heading to target_heading
     * Then sends these power levels to the motors.
     *
     * @param drive_x   X direction driving power (-1.0 to 1.0) +ve is forward
     * @param drive_y   Y direction driving power
     * @return          Current heading in degrees
     */
    public double xydrive(double drive_x, double drive_y) {
        // use turn PID controller to calculate turn input
        return xydrive(drive_x,drive_y,turn_to_target()); //
    }

    private double turn_to_target(double target_heading) {
        return 0d; // todo: write turn controller
    }
    private double turn_to_target() {
        return turn_to_target(target_heading);
    }

        /**
         * Pass the requested wheel motor powers to the appropriate hardware drive motors.
         *
         * @param alpha     Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
         * @param beta      Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
         * @param gamma     Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
         */
    public void setDrivePower(double alpha, double beta, double gamma) {
        // Output the values to the motor drives.
        alphaDrive.setPower(alpha);
        betaDrive.setPower(beta);
        gammaDrive.setPower(gamma);
    }

    /**
     * Calculates the motor powers required to achieve the requested
     * robot motions: Drive (Axial motion) and Turn (Yaw motion).
     * Then sends these power levels to the motors.
     * This function takes a single power and bearing rather than x and y
     *
     * @param power     driving power in selected direction (-1.0 to 1.0)
     * @param bearing   direction of movement in degrees
     * @param turn      Right/Left turning power (-1.0 to 1.0) +ve is CW
     * @return          Current heading in degrees
     */
    public double bearingdrive(double power, double bearing, double turn) {
        return xydrive(power * Math.sin(Math.toRadians(bearing)),power * Math.cos(Math.toRadians(bearing)),turn);
    }

    /**
     * Calculates the motor powers required to achieve the requested
     * robot motions: Drive (Axial motion) and Turn (Yaw motion).
     * This version uses PID control to create a turn command to control heading to target_heading
     * Then sends these power levels to the motors.
     * This function takes a single power and bearing rather than x and y
     *
     * @param power     driving power in selected direction (-1.0 to 1.0)
     * @param bearing   direction of movement in degrees
     * @return          Current heading in degrees
     */
    public double bearingdrive(double power, double bearing) {
        return bearingdrive(power,bearing,turn_to_target());
    }

    /**
     * Calculates the motor powers required to achieve the requested
     * robot motions: Drive (Axial motion) and Turn (Yaw motion).
     * This version uses PID control to travel in a straight line for a given distance
     * Then sends these power levels to the motors.
     * This function takes a distance and bearing rather than x and y, with power limited by max_speed
     *
     * @param distance  distance to travel, in inches
     * @param bearing   direction of movement in degrees
     * @param turn      Right/Left turning power (-1.0 to 1.0) +ve is CW
     * @return          Current heading in degrees
     */
    public double straightdrive(double distance, double bearing,double turn) {
        return bearingdrive(drive_target_distance(distance),bearing,turn);
    }

    /**
     * Calculates the motor powers required to achieve the requested
     * robot motions: Drive (Axial motion) and Turn (Yaw motion).
     * This version uses PID control to travel in a straight line for a given distance
     * This version also uses PID control to turn to a specified heading
     * Then sends these power levels to the motors.
     * This function takes a distance and bearing rather than x and y, with power limited by max_speed
     *
     * @param distance  distance to travel, in inches
     * @param bearing   direction of movement in degrees
     * @return          Current heading in degrees
     */
    public double straightdrive(double distance, double bearing) {
        return straightdrive(distance,bearing,turn_to_target());
    }
    private double drive_target_distance(double distance) {
        return 0.5d; // todo: write distance controller to drive linear_distance to target distance
    }

    public void stop() {
        xydrive(0,0,0);
    }
    /**
     * Reset yaw on IMU
     *
     */
    public void resetGyro() {
        imu.resetYaw();
    }

    public double[] getPosition() {
        return position;
    }

    public double[] getSpeed() {
        return speed;
    }

    public double getDistance() {
        return linear_distance;
    }

    /**
     * For convenience, resetDistance returns distance so you don't have to getDistance AND reset it
     * @return value of linear distance traveled prior to reset
     */
    public double resetDistance() {
        double tempdistance = linear_distance;
        linear_distance = 0;
        return tempdistance;
    }

    /**
     * Update position variables based on heading and travel of the drive wheels
     * @param sin Array of sin of angles of each wheel (alpha, beta, gamma) relative to heading 0
     * @param cos Array of sin of angles of each wheel (alpha, beta, gamma) relative to heading 0
     */
    private void updatePosition(double[] sin, double[] cos) {
        // temporarily grab the encoder count for each motor
        int[] newtravel = {alphaDrive.getCurrentPosition(), betaDrive.getCurrentPosition(), gammaDrive.getCurrentPosition()};
        double[] velocities = {alphaDrive.getVelocity(),betaDrive.getVelocity(),gammaDrive.getVelocity()};
        double distance_x = 0, distance_y = 0,speed_x = 0, speed_y = 0;
        /*
        A note about the math when calculating distance traveled:
            Each omni-wheel wheel spins freely in the direction perpendicular to its driven direction
            so we can't measure the distance traveled in the spinning-freely direction
            but we can figure it out by measuring the distance traveled by the other wheels
            In fact, if we add the distances traveled in a particular direction of all three wheels
            we will always get 1.5 times the robot's distance in that direction due to the fact
            that the wheels are 120 degrees separated from one another.

        We start with x and y distance of zero, as initialized above.
        Then, for i = 0, 1, and 2, add in the x and y components of distance traveled and multiply by 2/3
        Distance each wheel traveled is:
            Change in encoder count * pi * wheel diameter / ticks per rotation for drive gearmotor
            Then we multiply by 2/3 as described above
         We can do the same for speed in x and y directions by getting motor speed in ticks/second
         */
        for (int i = 0; i < 3; i++) {
            distance_x += sin[i] * (newtravel[i] - wheel_travel[i]);
            distance_y += cos[i] * (newtravel[i] - wheel_travel[i]);
            speed_x += sin[i] * velocities[i];
            speed_y += cos[i] * velocities[i];
        }
        distance_x *= (Math.PI * WHEEL_DIAM * 2 / 3 / DRIVE_TICKS_PER_ROT);
        distance_y *= (Math.PI * WHEEL_DIAM * 2 / 3 / DRIVE_TICKS_PER_ROT);
        speed_x *= (Math.PI * WHEEL_DIAM * 2 / 3 / DRIVE_TICKS_PER_ROT);
        speed_y *= (Math.PI * WHEEL_DIAM * 2 / 3 / DRIVE_TICKS_PER_ROT);
        position[0] += distance_x;
        position[1] += distance_y;
        speed[0] = speed_x;
        speed[1] = speed_y;
        // Estimate linear distance traveled using pythagoream theorum
        linear_distance += Math.hypot(distance_x,distance_y);
        // lastly we can save the current motor encoder counts for next time through the loop
        wheel_travel = newtravel;
    }

    public double getTarget_heading() {
        return target_heading;
    }

    public void setTarget_heading(double target_heading) {
        this.target_heading = target_heading; // todo: error checking on commanded target heading
    }

    public double getMax_speed() {
        return max_speed;
    }

    public void setMax_speed(double max_speed) {
        this.max_speed = max_speed; // todo: error checking on commanded max speed
    }


    /*
     *****
     * ARM RAISE/LOWER METHODS
     *****
     */

    /**
         * Pass the requested arm power to the appropriate hardware drive motor
         *
         * @param power driving power (-1.0 to 1.0)
         * @return      current encorder reading for this motor
         */
    public int setArmPower(double power) {
        // Scale the values so arm does not exceed maximum power
        if (power > 0) {
            arm.setPower(power*ARM_MAX_UP_POWER);
        } else {
            arm.setPower(power*ARM_MAX_DOWN_POWER);
        }
        int pos = arm.getCurrentPosition();
        return pos;
    }

    /*
     *****
     * END EFFECTOR METHODS
     *****
     */

    /**
     * Set turntable position.
     *
     * @param offset
     */
    public void setTTPosition(double offset) {
        offset = Range.clip(offset, -0.5, 0.5);
        turntable.setPosition(MID_SERVO + offset);
    }

    /**
     * Get turntable position.
     *
     * @return Offset (position relative to servo midpoint) for the turntable
     */
    public double getTTOffset() {
        return turntable.getPosition() - MID_SERVO;
    }


    /**
     * Set grabber_A position.
     *
     * @param offset
     */
    public void set_A_Position(double offset) {
        offset = Range.clip(offset, -0.5, 0.5);
        grabber_A.setPosition(MID_SERVO + offset);
    }

    /**
     * Get grabber_A position.
     *
     * @return Offset (position relative to servo midpoint) for Grabber A
     */
    public double getAOffset() {
        return grabber_A.getPosition() - MID_SERVO;
    }


    /**
     * Set grabber_B position.
     *
     * @param offset
     */
    public void set_B_Position(double offset) {
        offset = Range.clip(offset, -0.5, 0.5);
        grabber_B.setPosition(MID_SERVO + offset);
    }

    /**
     * Get grabber_B position.
     *
     * @return Offset (position relative to servo midpoint) for Grabber B
     */
    public double getBOffset() {
        return grabber_B.getPosition() - MID_SERVO;
    }

    /**
     * Pass the requested end effector power to the end effector servo
     *
     * @param power driving power (-1.0 to 1.0)
     * @return      voltage of angle feedback potentiometer on this end effector
     */
    public double setEEPower(double power) {
        endeffector.setPower(power);
        return endeffectorangle.getVoltage();
    }

    /**
     * Calculate target voltage on the end effector potentiometer based on arm height and desired angle
     * @param arm_height    encoder count for current or desired arm height
     * @param angle         desired angle relative to level (0 degrees)
     * @return              target voltage taking into account arm angle and ee angle relative to arm
     */
    public double get_eeLevelVoltage(int arm_height,double angle) {
        return v_level.getValue(arm_height) + (angle * VOLTS_PER_DEGREE);
    }


}
