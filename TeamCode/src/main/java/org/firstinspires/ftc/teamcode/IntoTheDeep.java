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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

// Use IntoTheDeep class for functions specific to IntoTheDeep 2024-2025 season mechanisms
public class IntoTheDeep extends HexbotHardware {
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
    private static double VOLTS_PER_DEGREE = 0.025; // just a guess todo: measure volts per degree for ee
    private LookupTable v_level;


    public IntoTheDeep(LinearOpMode opmode) {
        // run the super class constructor first
        super(opmode);
        // then initialize season-specific needs
        // Create a lookup table with:
        //      Key = Integer (encoder counts for arm motor)
        //      Value = Double (voltage on the potentiometer for the "level" zero position of end effector)
        v_level = new LookupTable();
        v_level.addRow(0,0.0d); // build lookuptable here todo: measure voltage at various arm positions
    }
    /**
     * Gets target voltage on the end effector potentiometer based on arm height and desired angle
     * @param arm_height    encoder count for current or desired arm height
     * @param angle         desired angle relative to level (0 degrees)
     * @return              target voltage taking into account arm angle and ee angle relative to arm
     */
    public double get_eeTargetVoltage(int arm_height,double angle) {
        return v_level.getValue(arm_height) + (angle * VOLTS_PER_DEGREE);
    }


}
