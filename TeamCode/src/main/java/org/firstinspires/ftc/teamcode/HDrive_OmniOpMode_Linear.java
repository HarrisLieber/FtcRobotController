package org.firstinspires.ftc.teamcode;/* Copyright (c) 2021 FIRST. All rights reserved.
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

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


/*
 *
 * This opmode, modeled on example BasicOmniOpMode, implements a basic H-drive for a 3-wheel
 * omni-directional robot. Robot has three omni-wheels spaced at 120 degrees.
 *
 */

@TeleOp(name="Basic: H Drive Linear OpMode", group="Linear OpMode")
// @Disabled
public class HDrive_OmniOpMode_Linear extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx backDrive = null;
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx arm = null;
    // Declare OpMode members for servos.
    private CRServo endeffector = null;
    private ServoImplEx turntable = null;
    private Servo grabber = null;
    // Declare OpMode members for sensors.
    private IMU imu = null;
    private AnalogInput endeffectorangle = null;
    // Declare constants for motion limitation
    private final double max_drive_power = 0.5d;
    private final double max_turn_power = 1.0d;
    private final double max_arm_power = 0.5d;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "drive_3");
        backDrive  = hardwareMap.get(DcMotorEx.class, "drive_2");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "drive_1");
        arm = hardwareMap.get(DcMotorEx.class, "arm_0");
        imu = hardwareMap.get(IMU.class, "imu");
        endeffector = hardwareMap.get(CRServo.class,"ee_0");
        endeffectorangle = hardwareMap.get(AnalogInput.class,"pot_0");
        turntable = hardwareMap.get(ServoImplEx.class,"tt_1");
        grabber = hardwareMap.get(Servo.class,"grabber_2");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        backDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.FORWARD);
        // Expand PWM Range to maxmimum supported by HS-485HB to extend rotation range
        turntable.setPwmRange(new PwmControl.PwmRange(553,2425));


        /* The next two lines define Hub orientation.
        */
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // H-drive Mode uses the left joystick to go forward and backward using left and right
            // drive motors, and the reat motor to turn.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            //double lateral =  gamepad1.left_stick_x;
            double lateral = 0.0d; // maintaining POV code below from BasicOmniOpMode example opmode
            double yaw     =  gamepad1.left_stick_x;
            double lift      = -gamepad1.right_stick_y;
            double swivel = 0.5;

            // Control end effector with right stick
            double deploy = gamepad1.right_stick_x;
            if (gamepad1.dpad_left) {
                swivel = 0.0d;
            }
            else if (gamepad1.dpad_right) {
                swivel = 1.0d;
            }
            else {
                swivel = 0.5d;
            }

            turntable.setPosition(swivel);


            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            //double leftFrontPower  = axial + lateral + yaw;
            //double rightFrontPower = axial - lateral - yaw;
            //double leftBackPower   = axial - lateral + yaw;
            //double rightBackPower  = axial + lateral - yaw;

            // Let's make this even simpler:
            double leftFrontPower = axial * max_drive_power;
            double rightFrontPower = axial * max_drive_power;
            double backPower = yaw * max_turn_power;
            double armPower = lift * max_arm_power;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            // max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            // max = Math.max(max, Math.abs(backPower));
            // max = Math.max(max, Math.abs(rightBackPower));

//             if (max > 1.0) {
//                leftFrontPower  /= max;
//                rightFrontPower /= max;
//                backPower   /= max;
//                rightBackPower  /= max;
//            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            backDrive.setPower(backPower);
            arm.setPower(armPower);
            endeffector.setPower(deploy);



            // Show the elapsed game time, potentiometer voltage, wheel power, and robot heading
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Pot voltage: ","%4.2f",endeffectorangle.getVoltage());
            // Check to see if heading reset is requested
            if (gamepad1.y) {
                telemetry.addData("Yaw", "Resetting\n");
                imu.resetYaw();
            } else {
                telemetry.addData("Yaw", "Press Y (triangle) on Gamepad to reset\n");
            }
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back (turning)", "%4.2f", backPower);

            // Retrieve Rotational Angles and Velocities
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            //telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
            //telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
            telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
            //telemetry.addData("Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate);
            //telemetry.addData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);

            telemetry.update();
        }
    }}
