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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.Range;



@TeleOp(name="Hexbot OmniTeleOp", group="Robot")
//@Disabled
public class OmniOpMode_hexbot extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    IntoTheDeep robot       = new IntoTheDeep(this);

    @Override
    public void runOpMode() {
        // drive (x,y) is desired drive power in x and y directions
        double[] drive = {0,0};
        // Position of robot relative to starting position (testing odometry)
        double[] position = {0,0};
        // Copies of the gamepad so we can detect rising/falling effects
        // gamepad[0] stores previous iteration's gamepad state, gamepad[1] is this iteration's
        Gamepad[] gamepad = new Gamepad[]{new Gamepad(), new Gamepad()};
        // turn power
        double turn = 0;
        final double TURN_POWER = 0.25; // creating a constant here for turn power
        double heading = 0;
        // arm power, arm encoder position, end effector servo power, and voltage of position feedback sensor
        double arm  = 0;
        int arm_pos = 0;
        //double ee   = 0;
        double ee_pos = 0;
        // angle (offset) of turntable, grabber a, grabber b
        double ttOffset   = 0;
        double aOffset = 0;
        double bOffset = 0;
        // store open and closed values - will capture these in constants later
        double aopen = 0, aclosed = 0, bopen = 0, bclosed = 0;
        // state for setting servo (0 = turntable, 1 = grabber A, 2 = grabber B)
        int servostate = 0;

        // initialize all the hardware, using the hardware class
        robot.init();

        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses START)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Store the previous state of the gamepad in gamepad[0] and current state in gamepad[1]
            gamepad[0].copy(gamepad[1]);
            gamepad[1].copy(gamepad1);

            // Run wheels in POV mode (note: The joystick goes negative when pushed forward, so negate it)
            // In this mode the Left stick moves the robot in XY directions, right/left dpad rotates.
            drive[0] = gamepad[1].right_stick_x;
            drive[1] = gamepad[1].right_stick_y;
            if (gamepad[1].dpad_left) turn = -TURN_POWER;
            else if (gamepad[1].dpad_right) turn = TURN_POWER;
            else turn = 0;

            // Use right stick Y axis to control arm
            arm = gamepad[1].left_stick_y;
            arm_pos = robot.setArmPower(arm);

            // Combine drive and turn for blended motion, get current heading
            heading = robot.xydrive(drive[1],drive[0], turn);
            position = robot.getPosition();

            // Use gamepad up & down dpad to raise and lower the end effector
            if (gamepad[1].dpad_up) ee_pos = robot.setEEPower(1.0);
            else if (gamepad[1].dpad_down) ee_pos = robot.setEEPower(-1.0);
            else ee_pos = robot.setEEPower(0.0);

            // detect rising edge on share button to change state
            if (gamepad[1].share && !gamepad[0].share) servostate = (servostate + 1) % 3;

            // use touchpad to set servo offset - for a and b grabbers, use lb and rb to store
            switch (servostate) {
                case 0:
                    // turntable
                    gamepad1.setLedColor(1,0,0,5000);
                    if (gamepad[1].touchpad_finger_1) ttOffset = gamepad[1].touchpad_finger_1_x;
                    robot.setTTPosition(ttOffset);
                    break;
                case 1:
                    // grabber A
                    gamepad1.setLedColor(0,1,0,5000);
                    if (gamepad[1].touchpad_finger_1) {
                        aOffset = gamepad[1].touchpad_finger_1_x;
                        robot.set_A_Position(aOffset);
                    } else if (gamepad[1].left_bumper && !gamepad[0].left_bumper) {
                        // detect rising edge on left bumper to store offset value
                        aclosed = aOffset;
                    } else if (gamepad[1].right_bumper && !gamepad[0].right_bumper) {
                        // detect rising edge on right bumper to store offset value
                        aclosed = aOffset;
                    }

                    break;
                case 2:
                    // grabber B
                    gamepad1.setLedColor(0,0,1,5000);
                    gamepad1.setLedColor(0,1,0,5000);
                    if (gamepad[1].touchpad_finger_1) {
                        bOffset = gamepad[1].touchpad_finger_1_x;
                        robot.set_B_Position(bOffset);
                    } else if (gamepad[1].left_bumper && !gamepad[0].left_bumper) {
                        // detect rising edge on left bumper to store offset value
                        bclosed = bOffset;
                    } else if (gamepad[1].right_bumper && !gamepad[0].right_bumper) {
                        // detect rising edge on right bumper to store offset value
                        bclosed = bOffset;
                    }
                    break;
            }

            // detect rising and falling edges on left and right triggers
            if (gamepad[1].right_trigger > 0.1 && gamepad[0].right_trigger <= 0.1) robot.set_A_Position(aclosed);
            else if (gamepad[1].right_trigger <= 0.1 && gamepad[0].right_trigger > 0.1) robot.set_A_Position(aopen);
            if (gamepad[1].left_trigger > 0.1 && gamepad[0].left_trigger <= 0.1) robot.set_B_Position(bclosed);
            else if (gamepad[1].left_trigger <= 0.1 && gamepad[0].left_trigger > 0.1) robot.set_B_Position(bopen);

            telemetry.addData("Field location (x,y): ","%4.1f,%4.1f",position[0],position[1]);
            telemetry.addData("Arm position: ",arm_pos);
            telemetry.addData("EE position (V): ","%1.3f",ee_pos);
            telemetry.addData("TT position: ","%1.3f",ttOffset);
            telemetry.addData("A open/closed: ","%1.3f,%1.3f",aopen,aclosed);
            telemetry.addData("B open/closed: ","%1.3f,%1.3f",bopen,bclosed);
            if (gamepad1.y) {
                robot.resetGyro();
            } else {
                telemetry.addData("Robot heading (Y to reset): ","%4.1f",heading);
            }

            telemetry.update();

            // Pace this loop so hands move at a reasonable speed.
            sleep(50);
        }
    }
}
