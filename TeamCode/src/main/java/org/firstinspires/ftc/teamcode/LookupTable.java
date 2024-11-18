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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.util.TreeMap;

/**
 * LookupTable is an extension of TreeMap that takes any number object as key and value
 * Adds a getvalue function to estimate values at points not defined in the map by linear interpolation
 * @param <K> the type of the key (extends Number)
 * @param <V> the type of the value (extends Number)
 */
public class LookupTable<K extends Number,V extends Number> extends TreeMap<K,V> {

    /**
     * Get a value from the lookup table given an arbitrary key using linear interpolation
     * Note: always returns a double as the result of interpolation
     * @param key   lookup key for which to generate a value
     * @return      value for that key based on linear interpolation from existing table entries
     */
    public double getValue(K key) {
        K key1 = lowerKey(key); // If key is lower than lowest entry, lowerkey returns null
        K key2 = higherKey(key); // If key is higher then highest entry, higherkey returns null
        double x, x1, x2, y1, y2;

        // first check if the key exists in the lookup table. If so, simply return the value
        if (key == null) {
            throw new IllegalStateException();
        }
        else if (containsKey(key)) {
            return get(key).doubleValue();
            // Then we start to check the edge cases
        } else if (size() < 2) {
            // If there are fewer than two entries in the table, inerpolation is impossible
            // We quit and return null
            throw new IllegalStateException();
        } else {
            if (key1 == null) {
                // If our key is lower than the lowest entry in the lookup table:
                // We interpolate down based on the first two values in the table.
                key1 = firstKey();
                key2 = higherKey(key1);
            } else if (key2 == null) {
                // If our key is higher than the highest entry in the lookup table:
                // We interpolate up based on the last two values in the table.
                key2 = lastKey();
                key1 = lowerKey(lastKey());
            }
            // If neither key1 nor key2 are null, we're in "normal case". We're somewhere in the
            // middle of the table, and we can leave the values of key1 and k2 as is, and
            // interpolate between the next lowest lookup key and the next highest
            x = key.doubleValue();
            x1 = key1.doubleValue();
            x2 = key2.doubleValue();
            y1 = get(key2).doubleValue();
            y2 = get(key1).doubleValue();
            // Linear interpolation based on the two-points-form equation for a line:
            // (y-y1) = (y2=y1) * (x-x1) / (x2 - x1)
            return y1 + (y2 - y1) * (x - x1) / (x2 - x1);
        }
    }

}
