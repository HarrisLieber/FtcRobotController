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


public class LookupTable {
    private TreeMap<Double,Double> l_Table;

    public LookupTable() {
        l_Table = new TreeMap<Double, Double>();
    }
    // Overload constructor to allow alternative method of creating lookup table
    public LookupTable(TreeMap<Double,Double> newtree) {
        l_Table = newtree;
    }


    /**
     * Add a row to the lookup table in key,value pairs (double, double)
     * @param key   double representing the lookup key for which to generate a value
     * @param value double representing the value associated with that key
     */
    public void addRow(double key, double value) {
        l_Table.put(key, value);
    }

    /**
     * Get a value from the lookup table given an arbitrary key using linear interpolation
     * @param key   double representing the lookup key for which to generate a value
     * @return      value for that key based on linear interpolation from existing table entries
     */
    public double getValue(double key) {
        double key1;
        double key2;

        // first check if the key exists in the lookup table. If so, simply return the value
        if (l_Table.containsKey(key)) {
            return l_Table.get(key);
            // Then we start to check the edge cases
        } else if (l_Table.size() < 2) {
            // If there are fewer than two entries in the table, inerpolation is impossible
            // We quit and return null
            throw new IllegalStateException();
        } else {
            if (key < l_Table.firstKey()) {
                // If our key is lower than the lowest entry in the lookup table:
                // We interpolate down based on the first two values in the table.
                key1 = l_Table.firstKey();
                key2 = l_Table.higherKey(key1);
                return l_Table.get(key1) - (key1-key) * (l_Table.get(key2)-l_Table.get(key1)) / (key2 - key1);

            } else if (key > l_Table.lastKey()) {
                // If our key is higher than the highest entry in the lookup table:
                // We interpolate up based on the last two values in the table.
                key2 = l_Table.lastKey();
                key1 = l_Table.lowerKey(key2);
                return l_Table.get(key2) + (key-key2) * (l_Table.get(key2)-l_Table.get(key1)) / (key2 - key1);

            } else {
                // Here's our normal case. We're somewhere in the middle of the table, and we
                // interpolate between the next lowest lookup key and the next highest
                key1 = l_Table.lowerKey(key);
                key2 = l_Table.higherKey(key);
                return l_Table.get(key1) + (key-key1) * (l_Table.get(key2)-l_Table.get(key1)) / (key2 - key1);
            }

        }
    }

}
