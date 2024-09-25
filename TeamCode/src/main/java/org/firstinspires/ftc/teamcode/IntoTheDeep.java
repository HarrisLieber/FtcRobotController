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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

// Use IntoTheDeep class for functions specific to IntoTheDeep 2024-2025 season mechanisms
public class IntoTheDeep extends HexbotHardware {
    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private NormalizedColorSensor color_A = null;
    private NormalizedColorSensor color_B = null;

    // Constants for each preset arm position so we can use them as indices for arrays of setpoints
    private static final int FLOOR = 0;
    private static final int WALL = 1;
    private static final int LOW_CHAMBER = 2;
    private static final int HIGH_CHAMBER = 3;
    private static final int LOW_BASKET = 4;
    private static final int HIGH_BASKET = 5;
    private static final int STOW = 6;
    // Setpoint tables for arm height and end effector angle
    private static int[] arm_pos = {0,0,0,0,0,0,0}; // todo: measure encoder ticks for each position
    private static double[] ee_angle = {0,75,90,90,45,45,150}; // just guesses todo: measure ee positions
    private static final double A_OPEN = 0; // todo: measure open and closed offsets for both grabbers
    private static final double A_CLOSED = 0;
    private static ColorMatch colorMatcher = new ColorMatch();
    private static final NormalizedRGBA TARGET_YELLOW = ColorMatch.makeColor(0.5f,0.5f,0f,1f);
    private static final NormalizedRGBA TARGET_BLUE = ColorMatch.makeColor(0f,0f,0.5f,1f);
    private static final NormalizedRGBA TARGET_RED = ColorMatch.makeColor(0.5f,0f,0f,1f);
    private static final NormalizedRGBA TARGET_FIELD = ColorMatch.makeColor(.1f,.1f,0.1f,.5f); // todo: measure target colors

    public IntoTheDeep(LinearOpMode opmode) {
        // run the super class constructor first
        super(opmode);
    }
    public void init() {
        super.init(); // run the normal hardware class init method
        // then initialize season-specific items

        // Define and Initialize Motors, Servos, Sensors
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        color_A  = myOpMode.hardwareMap.get(NormalizedColorSensor.class, "color_0");
        color_B  = myOpMode.hardwareMap.get(NormalizedColorSensor.class, "color_1");

        colorMatcher.addColorMatch(TARGET_YELLOW);
        colorMatcher.addColorMatch(TARGET_BLUE);
        colorMatcher.addColorMatch(TARGET_RED);
        colorMatcher.addColorMatch(TARGET_FIELD);
        colorMatcher.setConfidenceThreshold(0.9); // todo: test out confidence levels for color matching
    }

    public void sample_pickup() {
        set_A_Position(A_CLOSED);
        // Would be nice to be able to detect a successful pickup and auto-retry if not
    }

    public void sample_drop() {
        set_A_Position(A_OPEN);
    }

}
