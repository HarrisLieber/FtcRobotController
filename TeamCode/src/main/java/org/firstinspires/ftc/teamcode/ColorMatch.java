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

 * This class is modified version of ColorMatch class from Rev Robotics Color Sensor V3 source code:
 * https://github.com/REVrobotics/Color-Sensor-v3/blob/main/src/main/java/com/revrobotics/ColorMatch.java

 * Copyright (c) 2020 REV Robotics
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of REV Robotics nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import java.util.ArrayList;

public class ColorMatch {
    // HL: I think technically this might be a root mean square and not a Euclidean distance,
    // but I wasn't a math major. In any case, this method is left unaltered from REV Robotics code
    // except that Color class is replaced with NormalizedRGBA class from the FTC SDK
    private static double CalculateDistance(NormalizedRGBA color1, NormalizedRGBA color2) {
        double redDiff = color1.red - color2.red;
        double greenDiff = color1.green - color2.green;
        double blueDiff = color1.blue - color2.blue;

        return Math.sqrt((redDiff*redDiff + greenDiff*greenDiff + blueDiff*blueDiff)/2);
    }

    // Class fields and constructor left unaltered from REV Robotics source code except to replace
    // Color class with NormalizedRGBA Class from the FTC SDK
    private static final double kDefaultConfidence = 0.95;
    private double m_confidenceLevel;
    private ArrayList<NormalizedRGBA> m_colorsToMatch = new ArrayList<NormalizedRGBA>();

    public ColorMatch() {
        m_confidenceLevel = kDefaultConfidence;
    }


    // Left unaltered from REV Robotics source code except to replace Color Class with
    // FTC NormalizedRGBA Object
    /**
     * Add color to match object
     *
     * @param color value to add to matching
     *
     */
    public void addColorMatch(NormalizedRGBA color) {
        // add color to the arraylist of colors for matching
        m_colorsToMatch.add(color);
    }

    // Left unaltered from REV Robotics source code
    /**
     * Set the confidence interval for determining color. Defaults to 0.95
     *
     * @param confidence    A value between 0 and 1
     */
    public void setConfidenceThreshold(double confidence) {
        if (confidence < 0) {
            confidence = 0;
        } else if (confidence > 1) {
            confidence = 1;
        }
        m_confidenceLevel = confidence;
    }

    // Modified heavily from REV Robotics source code to change from creating an frc.Color object
    // from r g b values to creating an FTC NormalizedRGBA object from r g b a values
    // ALSO made this private instead of public, and added a call to this from addColorMatch
    /**
     * Create a NormalizedRGBA object from r g b and a
     *
     * @param r Red value
     * @param g Green value
     * @param b Blue value
     * @param a Alpha value
     *
     * @return  FTC NormalizedRGBA object
     */
    public static NormalizedRGBA makeColor(float r, float g, float b, float a) {
        NormalizedRGBA tempcolor = new NormalizedRGBA();
        tempcolor.red = r;
        tempcolor.green = g;
        tempcolor.blue = b;
        tempcolor.alpha = a;
        return tempcolor;
    }


    // Modified and combined two methods from REV Robotics source code
    // (original methods commented out and retained below for reference)
    /**
     * MatchColor uses euclidean distance to compare a given normalized RGB
     * vector against stored values
     *
     * @param color color to compare against stored colors
     *
     * @return  Closest color to match
     */
    public NormalizedRGBA matchColor(NormalizedRGBA color) {
        double minDistance = 1.0;
        int idx = 0;
        if (m_colorsToMatch.size() > 0) {
            for (int i = 0; i < m_colorsToMatch.size(); i++) {
                double targetDistance = CalculateDistance(m_colorsToMatch.get(i), color);
                if (targetDistance < minDistance) {
                    minDistance = targetDistance;
                    idx = i;
                }
            }
        } else {
            // if function is called with zero colors added to the list, return black
            return makeColor(0, 0, 0, 0);
        }
        // estimated confidence in color match is 1.0 - minDistance/total color magnitude
        if ((1.0 - (minDistance / (color.red + color.green + color.blue))) > m_confidenceLevel) {
            return m_colorsToMatch.get(idx);
        }
        // Following example of REV source, return null if we don't find a color with
        // confidence greater than set confidence threshold
        return null;
    }

    // Eliminated from REV Robotics source code and combined into one function above
    // Retained below for reference
//    /**
//     * MatchColor uses euclidean distance to compare a given normalized RGB
//     * vector against stored values
//     *
//     * @param colorToMatch color to compare against stored colors
//     *
//     * @return  Matched color if detected, returns null if no color detected
//     * confidence level
//     */
//    public ColorMatchResult matchColor(Color colorToMatch) {
//        ColorMatchResult match = matchClosestColor(colorToMatch);
//        if ( match.confidence > m_confidenceLevel ) {
//            return match;
//        }
//        return null;
//    }
//    /**
//     * MatchColor uses euclidean distance to compare a given normalized RGB
//     * vector against stored values
//     *
//     * @param color color to compare against stored colors
//     *
//     * @return  Closest color to match
//     */
//    public ColorMatchResult matchClosestColor(Color color) {
//        double magnitude = color.red + color.blue + color.green;
//        if (magnitude > 0 && m_colorsToMatch.size() > 0) {
//            double minDistance = 1.0;
//            int idx = 0;
//
//            for (int i=0; i < m_colorsToMatch.size(); i++) {
//                double targetDistance = CalculateDistance(m_colorsToMatch.get(i), color);
//
//                if (targetDistance < minDistance) {
//                    minDistance = targetDistance;
//                    idx = i;
//                }
//            }
//            ColorMatchResult match = new ColorMatchResult(m_colorsToMatch.get(idx), 1.0 - (minDistance / magnitude) );
//            return match;
//        } else {
//            //return frc::Color::kBlack;
//            return new ColorMatchResult(Color.kBlack, 0.0);
//        }
//    }
}
