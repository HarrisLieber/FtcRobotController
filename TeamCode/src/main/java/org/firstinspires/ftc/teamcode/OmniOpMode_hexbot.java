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
        // turn power
        double turn = 0;
        double heading = 0;
        // arm power and end effector servo power
        double arm  = 0;
        int arm_pos = 0;
        double ee   = 0;
        double ee_pos = 0;
        // angle (offset) of turntable
        double ttOffset   = 0;

        // initialize all the hardware, using the hardware class
        robot.init();

        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses START)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forward, so negate it)
            // In this mode the Left stick moves the robot in XY directions, right stick rotates.
            drive[0] = gamepad1.right_stick_x;
            drive[1] = gamepad1.right_stick_y;
            turn  =  gamepad1.left_stick_x;

            // Use right stick Y axis to control arm
            arm = gamepad1.left_stick_y;
            arm_pos = robot.setArmPower(arm);

            // Combine drive and turn for blended motion, get current heading
            heading = robot.xydrive(drive[1],drive[0], turn);

            // Use gamepad left & right dpad to swivel turntable
            // Each time around the loop, the servo will move by a small amount.
            // Limit the total offset to half of the full travel range
            if (gamepad1.dpad_right)
                ttOffset += robot.TT_SPEED;
            else if (gamepad1.dpad_left)
                ttOffset -= robot.TT_SPEED;
            // Move turntable to new position
            robot.setTTPosition(ttOffset);

            // Use gamepad up & down dpad to raise and lower the end effector
            if (gamepad1.dpad_up) {
                ee_pos = robot.setEEPower(1.0);
            } else if (gamepad1.dpad_down) {
                ee_pos = robot.setEEPower(-1.0);
            } else {
                ee_pos = robot.setEEPower(0.0);
            }

            telemetry.addData("Arm position: ",arm_pos);
            telemetry.addData("EE position (V): ","%1.3f",ee_pos);
            telemetry.addData("TT position: ","%1.3f",ttOffset);
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
