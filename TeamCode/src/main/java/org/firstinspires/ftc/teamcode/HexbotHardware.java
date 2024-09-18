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
    private Servo grabber = null;
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
    public static final double MID_SERVO       =  0.5 ;
    public static final double TT_SPEED      =  0.02 ;  // sets rate to move turntable servo

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public HexbotHardware(LinearOpMode opmode) {
        myOpMode = opmode;
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
        grabber = myOpMode.hardwareMap.get(Servo.class,"grabber_2");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        alphaDrive.setDirection(DcMotor.Direction.REVERSE);
        gammaDrive.setDirection(DcMotor.Direction.REVERSE);
        betaDrive.setDirection(DcMotor.Direction.REVERSE);
        arm.setDirection(DcMotor.Direction.REVERSE);

        // Expand PWM Range to maximum supported by HS-485HB to extend rotation range
        turntable.setPwmRange(new PwmControl.PwmRange(553,2425));

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        alphaDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        gammaDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        betaDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
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
     * @param Drive_x   X direction driving power (-1.0 to 1.0) +ve is forward
     * @param Drive_y   Y direction driving power
     * @param Turn      Right/Left turning power (-1.0 to 1.0) +ve is CW
     * @return          Current heading in degrees
     */
    public double xydrive(double Drive_x, double Drive_y, double Turn) {
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
        double F_alpha = (Drive_x*(cos[1]-cos[2])+Drive_y*(sin[2]-sin[1])+Turn*(sin[1]*cos[2]-sin[2]*cos[1]))/determinant;
        double F_beta = (Drive_x*(cos[2]-cos[0])+Drive_y*(sin[0]-sin[2])+Turn*(sin[2]*cos[0]-sin[0]*cos[2]))/determinant;
        double F_gamma = (Drive_x*(cos[0]-cos[1])+Drive_y*(sin[1]-sin[0])+Turn*(sin[0]*cos[1]-sin[1]*cos[0]))/determinant;

        // Scale the values so none exceed maximum drive power
        double max = Math.max(Math.max(Math.abs(F_alpha), Math.abs(F_beta)), Math.abs(F_gamma));
        if (max > MAX_DRIVE_POWER) {
            F_alpha /= max;
            F_beta /= max;
            F_gamma /= max;
        }

        // Use existing function to drive three wheels.
        setDrivePower(F_alpha,F_beta,F_gamma);
        // return current heading
        return orientation.getYaw(AngleUnit.DEGREES);
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
     * Reset yaw on IMU
     *
     */
    public void resetGyro() {
        imu.resetYaw();
    }

}
