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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;


@TeleOp(name="Test new gamepad functions", group="Robot")
//@Disabled
public class TestGamepadEx extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    IntoTheDeep robot       = new IntoTheDeep(this);

    @Override
    public void runOpMode() {
        GamepadEx gamepad = new GamepadEx(gamepad1);
        // drive (x,y) is desired drive power in x and y directions
        double[] drive = {0,0};
        // turn power
        double turn = 0;
        double heading = 0;
        double directionpressed = 0;

        // initialize all the hardware, using the hardware class
        robot.init();

        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses START)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Update the gamepad state once per loop
            gamepad.update();


            // Run wheels in POV mode
            // In this mode the right stick moves the robot in XY directions, left stick turns
            drive[0] = gamepad.get(GamepadEx.Axis.RIGHT_X);
            drive[1] = gamepad.get(GamepadEx.Axis.RIGHT_Y);
            turn = gamepad.get(GamepadEx.Axis.LEFT_X);
            // Test dpad rising edge detection
            if (gamepad.dpad_rising != HexbotHardware.Dmap.NONE) {
                // Android studio warns unboxing may produce NPE as EnumMap returns wrapper object
                // Double and we are assigning to a double - should not be an issue in practice
                heading = HexbotHardware.DIRECTION.get(gamepad.dpad_rising);
            }
            // Test dpad press detection
            if (gamepad.dpad_pressed != HexbotHardware.Dmap.NONE) {
                // Android studio warns unboxing may produce NPE as EnumMap returns wrapper object
                // Double and we are assigning to a double - should not be an issue in practice
                directionpressed = HexbotHardware.DIRECTION.get(gamepad.dpad_pressed);
            }
            // Test button field rising edge detection
            switch (gamepad.buttons_rising) {
                case NORTH:
                    gamepad.rumble(0.5,0.5,250);
                    break;
                case NORTHEAST:
                    gamepad.rumble(0.25,0.75,250);
                    break;
                case EAST:
                    gamepad.rumble(0.0,1.0,250);
                    break;
                case SOUTHEAST:
                    gamepad.rumble(0.125,0.625,375);
                    break;
                case SOUTH:
                    gamepad.rumble(0.25,0.25,500);
                    break;
                case SOUTHWEST:
                    gamepad.rumble(0.625,0.125,375);
                    break;
                case WEST:
                    gamepad.rumble(1.0,0.0,250);
                    break;
                case NORTHWEST:
                    gamepad.rumble(0.75,0.25,250);
                    break;
                case NORTHSOUTH:
                    gamepad.rumbleBlips(2);
                    break;
                case WESTEAST:
                    gamepad.rumbleBlips(3);
                    break;
                case NONE:
                    break;
                default:
                    throw new IllegalStateException("Unexpected value: " + gamepad.buttons_rising);
            }
            // Test button field press detection
            switch (gamepad.buttons_pressed) {
                case NORTH:
                    gamepad.setLedColor(0.0,1.0,0.0,Gamepad.LED_DURATION_CONTINUOUS);
                    break;
                case NORTHEAST:
                    gamepad.setLedColor(0.5,0.5,0.0,Gamepad.LED_DURATION_CONTINUOUS);
                    break;
                case EAST:
                    gamepad.setLedColor(1.0,0.0,0.0,Gamepad.LED_DURATION_CONTINUOUS);
                    break;
                case SOUTHEAST:
                    gamepad.setLedColor(0.75,0.0,0.5,Gamepad.LED_DURATION_CONTINUOUS);
                    break;
                case SOUTH:
                    gamepad.setLedColor(0.0,0.0,1.0,Gamepad.LED_DURATION_CONTINUOUS);
                    break;
                case SOUTHWEST:
                    gamepad.setLedColor(0.5,0.0,1.0,Gamepad.LED_DURATION_CONTINUOUS);
                    break;
                case WEST:
                    gamepad.setLedColor(1.0,0.0,1.0,Gamepad.LED_DURATION_CONTINUOUS);
                    break;
                case NORTHWEST:
                    gamepad.setLedColor(0.5,0.5,0.5,Gamepad.LED_DURATION_CONTINUOUS);
                    break;
                case NORTHSOUTH:
                    gamepad.setLedColor(1.0,1.0,1.0,Gamepad.LED_DURATION_CONTINUOUS);
                    break;
                case WESTEAST:
                    gamepad.setLedColor(1.0,1.0,1.0,Gamepad.LED_DURATION_CONTINUOUS);
                    break;
                case NONE:
                    gamepad.setLedColor(0.0,0.0,0.0,Gamepad.LED_DURATION_CONTINUOUS);
                    break;
                default:
                    throw new IllegalStateException("Unexpected value: " + gamepad.buttons_rising);
            }


            telemetry.addData("Drive (x,y): ","%1.3f,%1.3f",drive[0],drive[1]);
            telemetry.addData("Turn: ","1.3f",turn);
            telemetry.addData("Heading: ","%4.1f",heading);
            telemetry.addData("Pressed: ","%4.1f",directionpressed);

            telemetry.update();


            // Pace this loop so hands move at a reasonable speed.
            sleep(50);
        }

    }
}
