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

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadEx extends Gamepad {

    /**
     * The gamepad object to be passed from the opmode
     */
    public Gamepad gamepad;
    private Gamepad current; // current values of the gamepad
    private Gamepad past; // gamepad values from past iteration
    public HexbotHardware.Dmap dpad_pressed, dpad_rising, buttons_pressed, buttons_rising;

    /**
     * threshold value for treating a trigger as a button
     */
    private float triggerthreshold;
    private static final float TRIGGERTHRESHOLDDEFAULT = 0.2F;

    public float getTriggerthreshold() {
        return triggerthreshold;
    }

    public void setTriggerthreshold(float triggerthreshold) {
        this.triggerthreshold = triggerthreshold;
    }

    /**
     * Enumerators for the keys, joystick axes, and touchpad axes on the controller
     * Also including TRIGGER_BUTTON_L and _R for treating triggers as buttons
     * Aliases for Xbox and PS controllers from the FTC SDK:
     * circle = b
     * cross = a
     * triangle = y
     * square = x
     * share = back
     * options = start
     * ps = guide
     */
    public enum Button {
        Y, X, A, B, TRIANGLE, SQUARE, CROSS, CIRCLE,
        BACK, START, GUIDE, SHARE, OPTIONS, PS,
        BUMPER_L, BUMPER_R, STICK_L, STICK_R, TOUCHPAD_1, TOUCHPAD_2,
        DPAD_UP, DPAD_DOWN, DPAD_LEFT, DPAD_RIGHT,
        TRIGGER_BUTTON_L, TRIGGER_BUTTON_R
    }

    public enum Trigger {
        TRIGGER_L, TRIGGER_R
    }

    public enum Axis {
        LEFT_X, LEFT_Y, RIGHT_X, RIGHT_Y,
        TOUCHPAD_1_X, TOUCHPAD_1_Y, TOUCHPAD_2_X, TOUCHPAD_2_Y
    }

//    /**
//     * Enumerator for direction map for dpad and button field
//     * Convenient way to report out common buttons/combos of buttons
//     * Probably doesn't really make sense for this to live inside of GamePadEx
//     */
//    public enum Dmap {
//        NORTH, NORTHEAST, EAST, SOUTHEAST, SOUTH, SOUTHWEST, WEST, NORTHWEST,
//        NORTHSOUTH, WESTEAST, NONE
//    }

    /**
     *
     * Constructor containing the gamepad passed from the opmode
     *
     * @param gamepad the gamepad object from the opmode
     */
    public GamepadEx(Gamepad gamepad) {
        this.gamepad = gamepad;
        current.copy(this.gamepad);
        triggerthreshold = TRIGGERTHRESHOLDDEFAULT;
    }

    /**
     * Create a single value for the dpad that recognizes any single keypress or combination of two
     * This method works around the dpad roughly clockwise but it will recognize any two-button combination.
     * It will not recognize any three-button combinations
     */
    private void updateDpad() {
        if (current.dpad_up) {
            if (current.dpad_right) {
                dpad_pressed = HexbotHardware.Dmap.NORTHEAST;
                if (!past.dpad_right || !past.dpad_up) dpad_rising = HexbotHardware.Dmap.NORTHEAST;
                else dpad_rising = HexbotHardware.Dmap.NONE;
            }
            else if (current.dpad_left) {
                dpad_pressed = HexbotHardware.Dmap.NORTHWEST;
                if (!past.dpad_left || !past.dpad_up) dpad_rising = HexbotHardware.Dmap.NORTHWEST;
                else dpad_rising = HexbotHardware.Dmap.NONE;
            }
            else if (current.dpad_down) {
                dpad_pressed = HexbotHardware.Dmap.NORTHSOUTH;
                if (!past.dpad_down || !past.dpad_up) dpad_rising = HexbotHardware.Dmap.NORTHSOUTH;
                else dpad_rising = HexbotHardware.Dmap.NONE;
            }
            else {
                dpad_pressed = HexbotHardware.Dmap.NORTH;
                if (!past.dpad_up) dpad_rising = HexbotHardware.Dmap.NORTH;
                else dpad_rising = HexbotHardware.Dmap.NONE;
            }
        }
        else if (current.dpad_right) {
            if (current.dpad_down) {
                dpad_pressed = HexbotHardware.Dmap.SOUTHEAST;
                if (!past.dpad_right || !past.dpad_down) dpad_rising = HexbotHardware.Dmap.SOUTHEAST;
                else dpad_rising = HexbotHardware.Dmap.NONE;
            }
            else if (current.dpad_left) {
                dpad_pressed = HexbotHardware.Dmap.WESTEAST;
                if (!past.dpad_right || !past.dpad_left) dpad_rising = HexbotHardware.Dmap.WESTEAST;
                else dpad_rising = HexbotHardware.Dmap.NONE;
            }
            else {
                dpad_pressed = HexbotHardware.Dmap.EAST;
                if (!past.dpad_right) dpad_rising = HexbotHardware.Dmap.EAST;
                else dpad_rising = HexbotHardware.Dmap.NONE;
            }
        }
        else if (current.dpad_down) {
            if (current.dpad_left) {
                dpad_pressed = HexbotHardware.Dmap.SOUTHWEST;
                if (!past.dpad_left || !past.dpad_down) dpad_rising = HexbotHardware.Dmap.SOUTHWEST;
                else dpad_rising = HexbotHardware.Dmap.NONE;
            }
            else {
                dpad_pressed = HexbotHardware.Dmap.SOUTH;
                if (!past.dpad_down) dpad_rising = HexbotHardware.Dmap.SOUTH;
                else dpad_rising = HexbotHardware.Dmap.NONE;
            }
        }
        else if (current.dpad_left) {
            dpad_pressed = HexbotHardware.Dmap.WEST;
            if (!past.dpad_left) dpad_rising = HexbotHardware.Dmap.WEST;
            else dpad_rising = HexbotHardware.Dmap.NONE;
        }
        else {
            dpad_pressed = HexbotHardware.Dmap.NONE;
            dpad_rising = HexbotHardware.Dmap.NONE;
        }
    }

    /**
     * Create a single value for the button field that recognizes any single keypress or combination of two
     * This method works around the field roughly clockwise but it will recognize any two-button combination.
     * It will not recognize any three-button combinations
     */
    private void updateButtonmap() {
        if (current.y) {
            if (current.b) {
                buttons_pressed = HexbotHardware.Dmap.NORTHEAST;
                if (!past.y || !past.b) buttons_rising = HexbotHardware.Dmap.NORTHEAST;
                else buttons_rising = HexbotHardware.Dmap.NONE;
            }
            else if (current.x) {
                buttons_pressed = HexbotHardware.Dmap.NORTHWEST;
                if (!past.x || !past.y) buttons_rising = HexbotHardware.Dmap.NORTHWEST;
                else buttons_rising = HexbotHardware.Dmap.NONE;
            }
            else if (current.a) {
                buttons_pressed = HexbotHardware.Dmap.NORTHSOUTH;
                if (!past.x || !past.a) buttons_rising = HexbotHardware.Dmap.NORTHSOUTH;
                else buttons_rising = HexbotHardware.Dmap.NONE;
            }
            else {
                buttons_pressed = HexbotHardware.Dmap.NORTH;
                if (!past.y) buttons_rising = HexbotHardware.Dmap.NORTH;
                else buttons_rising = HexbotHardware.Dmap.NONE;
            }
        }
        else if (current.b) {
            if (current.a) {
                buttons_pressed = HexbotHardware.Dmap.SOUTHEAST;
                if (!past.b || !past.a) buttons_rising = HexbotHardware.Dmap.SOUTHEAST;
                else buttons_rising = HexbotHardware.Dmap.NONE;
            }
            else if (current.x) {
                buttons_pressed = HexbotHardware.Dmap.WESTEAST;
                if (!past.b || !past.x) buttons_rising = HexbotHardware.Dmap.WESTEAST;
                else buttons_rising = HexbotHardware.Dmap.NONE;
            }
            else {
                buttons_pressed = HexbotHardware.Dmap.EAST;
                if (!past.b) buttons_rising = HexbotHardware.Dmap.EAST;
                else buttons_rising = HexbotHardware.Dmap.NONE;
            }
        }
        else if (current.a) {
            if (current.x) {
                buttons_pressed = HexbotHardware.Dmap.SOUTHWEST;
                if (!past.x || !past.a) buttons_rising = HexbotHardware.Dmap.SOUTHWEST;
                else buttons_rising = HexbotHardware.Dmap.NONE;
            }
            else {
                buttons_pressed = HexbotHardware.Dmap.SOUTH;
                if (!past.a) buttons_rising = HexbotHardware.Dmap.SOUTH;
                else buttons_rising = HexbotHardware.Dmap.NONE;
            }
        }
        else if (current.x) {
            buttons_pressed = HexbotHardware.Dmap.WEST;
            if (!past.x) buttons_rising = HexbotHardware.Dmap.WEST;
            else buttons_rising = HexbotHardware.Dmap.NONE;
        }
        else {
            buttons_pressed = HexbotHardware.Dmap.NONE;
            buttons_rising = HexbotHardware.Dmap.NONE;
        }
    }


    /**
     *
     * Updates current and past versions of the gamepad for rising and falling edge detection
     * Must be called once in the main loop
     */
    public void update() {
        past.copy(current); // capture state of gamepad from previous loop
        current.copy(gamepad); // capture current state of gamepad at beginning of loop
        updateDpad();
        updateButtonmap();
        }

    /**
     * get current value of specified button
     * @param button GamepadEx.Button enumerator representing button to get value of
     * @return true if button is down, false if button is not pressed (including touchpad virtual buttons)
     */
    public boolean get(Button button) {
        boolean returnvalue = false;
        switch (button) {
            case Y:
                returnvalue = current.y;
                break;
            case X:
                returnvalue = current.x;
                break;
            case A:
                returnvalue = current.a;
                break;
            case B:
                returnvalue = current.b;
                break;
            case TRIANGLE:
                returnvalue = current.triangle;
                break;
            case SQUARE:
                returnvalue = current.square;
                break;
            case CROSS:
                returnvalue = current.cross;
                break;
            case CIRCLE:
                returnvalue = current.circle;
                break;
            case BACK:
                returnvalue = current.back;
                break;
            case START:
                returnvalue = current.start;
                break;
            case GUIDE:
                returnvalue = current.guide;
                break;
            case SHARE:
                returnvalue = current.share;
                break;
            case OPTIONS:
                returnvalue = current.options;
                break;
            case PS:
                returnvalue = current.ps;
                break;
            case BUMPER_L:
                returnvalue = current.left_bumper;
                break;
            case BUMPER_R:
                returnvalue = current.right_bumper;
                break;
            case STICK_L:
                returnvalue = current.left_stick_button;
                break;
            case STICK_R:
                returnvalue = current.right_stick_button;
                break;
            case TOUCHPAD_1:
                returnvalue = current.touchpad_finger_1;
                break;
            case TOUCHPAD_2:
                returnvalue = current.touchpad_finger_2;
                break;
            case DPAD_UP:
                returnvalue = current.dpad_up;
                break;
            case DPAD_DOWN:
                returnvalue = current.dpad_down;
                break;
            case DPAD_LEFT:
                returnvalue = current.dpad_left;
                break;
            case DPAD_RIGHT:
                returnvalue = current.dpad_right;
                break;
            case TRIGGER_BUTTON_L:
                returnvalue = (current.left_trigger >= triggerthreshold);
                break;
            case TRIGGER_BUTTON_R:
                returnvalue = (current.right_trigger >= triggerthreshold);
                break;
        }
        return returnvalue;
    }

    /**
     * get current value of a pair of buttons (button1 AND button2)
     * @param button1 GamepadEx.Button enumerator representing button to get value of
     * @param button2 GamepadEx.Button enumerator representing button to get value of
     * @return true if both buttons are down, false if one or neither is pressed
     */
    public boolean get(Button button1,Button button2) {
        return (get(button1) && get(button2));
    }

    /**
     * get current value of specified trigger
     * @param trigger GamepadEx.Trigger enumerator representing trigger to get value of
     * @return float between 0.0 if trigger is not pressed and 1.1 if fully pressed
     */
    public float get(Trigger trigger) {
        float returnvalue = 0.0f;
        switch (trigger) {
            case TRIGGER_L:
                returnvalue = current.left_trigger;
                break;
            case TRIGGER_R:
                returnvalue = current.right_trigger;
                break;
        }
        return returnvalue;
    }

    /**
     * get current value of specified axis (joystick or touchpad)
     * @param axis GamepadEx.Axis enumerator representing axis to get value of
     * @return float between -1.0 if joystick is all the way left and +1.0 if joystick is all the way right
     */
    public float get(Axis axis) {
        float returnvalue = 0.0f;
        switch (axis) {
            case LEFT_X:
                returnvalue = current.left_stick_x;
                break;
            case LEFT_Y:
                returnvalue = current.left_stick_y;
                break;
            case RIGHT_X:
                returnvalue = current.right_stick_x;
                break;
            case RIGHT_Y:
                returnvalue = current.right_stick_y;
                break;
            case TOUCHPAD_1_X:
                returnvalue = current.touchpad_finger_1_x;
                break;
            case TOUCHPAD_1_Y:
                returnvalue = current.touchpad_finger_1_y;
                break;
            case TOUCHPAD_2_X:
                returnvalue = current.touchpad_finger_2_x;
                break;
            case TOUCHPAD_2_Y:
                returnvalue = current.touchpad_finger_2_y;
                break;
        }
        return returnvalue;
    }

    /**
     * get past value of specified button
     * @param button GamepadEx.Button enumerator representing button to get value of
     * @return true if button is down, false if button is not pressed (including touchpad virtual buttons)
     */
    public boolean getpast(Button button) {
        boolean returnvalue = false;
        switch (button) {
            case Y:
                returnvalue = past.y;
                break;
            case X:
                returnvalue = past.x;
                break;
            case A:
                returnvalue = past.a;
                break;
            case B:
                returnvalue = past.b;
                break;
            case TRIANGLE:
                returnvalue = past.triangle;
                break;
            case SQUARE:
                returnvalue = past.square;
                break;
            case CROSS:
                returnvalue = past.cross;
                break;
            case CIRCLE:
                returnvalue = past.circle;
                break;
            case BACK:
                returnvalue = past.back;
                break;
            case START:
                returnvalue = past.start;
                break;
            case GUIDE:
                returnvalue = past.guide;
                break;
            case SHARE:
                returnvalue = past.share;
                break;
            case OPTIONS:
                returnvalue = past.options;
                break;
            case PS:
                returnvalue = past.ps;
                break;
            case BUMPER_L:
                returnvalue = past.left_bumper;
                break;
            case BUMPER_R:
                returnvalue = past.right_bumper;
                break;
            case STICK_L:
                returnvalue = past.left_stick_button;
                break;
            case STICK_R:
                returnvalue = past.right_stick_button;
                break;
            case TOUCHPAD_1:
                returnvalue = past.touchpad_finger_1;
                break;
            case TOUCHPAD_2:
                returnvalue = past.touchpad_finger_2;
                break;
            case DPAD_UP:
                returnvalue = past.dpad_up;
                break;
            case DPAD_DOWN:
                returnvalue = past.dpad_down;
                break;
            case DPAD_LEFT:
                returnvalue = past.dpad_left;
                break;
            case DPAD_RIGHT:
                returnvalue = past.dpad_right;
                break;
            case TRIGGER_BUTTON_L:
                returnvalue = (past.left_trigger >= triggerthreshold);
                break;
            case TRIGGER_BUTTON_R:
                returnvalue = (past.right_trigger >= triggerthreshold);
                break;
        }
        return returnvalue;
    }

    /**
     * get past value of a pair of buttons (button1 AND button2)
     * @param button1 GamepadEx.Button enumerator representing button to get value of
     * @param button2 GamepadEx.Button enumerator representing button to get value of
     * @return true if both buttons are down, false if one or neither is pressed
     */
    public boolean getpast(Button button1,Button button2) {
        return (getpast(button1) && getpast(button2));
    }

    /**
     * get past value of specified trigger
     * @param trigger GamepadEx.Trigger enumerator representing trigger to get value of
     * @return float between 0.0 if trigger is not pressed and 1.1 if fully pressed
     */
    public float getpast(Trigger trigger) {
        float returnvalue = 0.0f;
        switch (trigger) {
            case TRIGGER_L:
                returnvalue = past.left_trigger;
                break;
            case TRIGGER_R:
                returnvalue = past.right_trigger;
                break;
        }
        return returnvalue;
    }

    /**
     * get past value of specified axis (joystick or touchpad)
     * @param axis GamepadEx.Axis enumerator representing axis to get value of
     * @return float between -1.0 if joystick is all the way left and +1.0 if joystick is all the way right
     */
    public float getpast(Axis axis) {
        float returnvalue = 0.0f;
        switch (axis) {
            case LEFT_X:
                returnvalue = past.left_stick_x;
                break;
            case LEFT_Y:
                returnvalue = past.left_stick_y;
                break;
            case RIGHT_X:
                returnvalue = past.right_stick_x;
                break;
            case RIGHT_Y:
                returnvalue = past.right_stick_y;
                break;
            case TOUCHPAD_1_X:
                returnvalue = past.touchpad_finger_1_x;
                break;
            case TOUCHPAD_1_Y:
                returnvalue = past.touchpad_finger_1_y;
                break;
            case TOUCHPAD_2_X:
                returnvalue = past.touchpad_finger_2_x;
                break;
            case TOUCHPAD_2_Y:
                returnvalue = past.touchpad_finger_2_y;
                break;
        }
        return returnvalue;
    }


    /**
     * detect rising edge of a button
     * @param button GamepadEx.Button enumerator representing button to get value of
     * @return true if button was just pressed (i.e. pressed now and unpressed in previous loop)
     */
    public boolean risingedge(Button button) {
        boolean returnvalue = false;
        switch (button) {
            case Y:
                returnvalue = current.y && !past.y;
                break;
            case X:
                returnvalue = current.x && !past.x;
                break;
            case A:
                returnvalue = current.a && !past.a;
                break;
            case B:
                returnvalue = current.b && !past.b;
                break;
            case TRIANGLE:
                returnvalue = current.triangle && !past.triangle;
                break;
            case SQUARE:
                returnvalue = current.square && !past.square;
                break;
            case CROSS:
                returnvalue = current.cross && !past.cross;
                break;
            case CIRCLE:
                returnvalue = current.circle && !past.circle;
                break;
            case BACK:
                returnvalue = current.back && !past.back;
                break;
            case START:
                returnvalue = current.start && !past.start;
                break;
            case GUIDE:
                returnvalue = current.guide && !past.guide;
                break;
            case SHARE:
                returnvalue = current.share && !past.share;
                break;
            case OPTIONS:
                returnvalue = current.options && !past.options;
                break;
            case PS:
                returnvalue = current.ps && !past.ps;
                break;
            case BUMPER_L:
                returnvalue = current.left_bumper && !past.left_bumper;
                break;
            case BUMPER_R:
                returnvalue = current.right_bumper && !past.right_bumper;
                break;
            case STICK_L:
                returnvalue = current.left_stick_button && !past.left_stick_button;
                break;
            case STICK_R:
                returnvalue = current.right_stick_button && !past.right_stick_button;
                break;
            case TOUCHPAD_1:
                returnvalue = current.touchpad_finger_1 && !past.touchpad_finger_1;
                break;
            case TOUCHPAD_2:
                returnvalue = current.touchpad_finger_2 && !past.touchpad_finger_2;
                break;
            case DPAD_UP:
                returnvalue = current.dpad_up && !past.dpad_up;
                break;
            case DPAD_DOWN:
                returnvalue = current.dpad_down && !past.dpad_down;
                break;
            case DPAD_LEFT:
                returnvalue = current.dpad_left && !past.dpad_left;
                break;
            case DPAD_RIGHT:
                returnvalue = current.dpad_right && !past.dpad_right;
                break;
            case TRIGGER_BUTTON_L:
                returnvalue = (current.left_trigger >= triggerthreshold) && (past.left_trigger < triggerthreshold);
                break;
            case TRIGGER_BUTTON_R:
                returnvalue = (current.right_trigger >= triggerthreshold) && (past.right_trigger < triggerthreshold);
                break;
        }
        return returnvalue;
    }

    /**
     * detect rising edge of a pair of buttons (both pressed and at least one with rising edge)
     * @param button1 GamepadEx.Button enumerator representing button to get value of
     * @param button2 GamepadEx.Button enumerator representing button to get value of
     * @return true if both buttons are down and at least one has rising edge
     */
    public boolean risingedge(Button button1,Button button2) {
        boolean returnvalue = false;
        if (risingedge(button1)) returnvalue = get(button2);
        else if (risingedge(button2)) returnvalue = get(button1);
        return returnvalue;
    }

    /**
     * detect rising edge of a trigger
     * this is identical to the code for treating a trigger as a button
     * this method provided as a convenient alternate way to access that logic
     * @param trigger GamepadEx.Trigger enumerator representing trigger to get value of
     * @return true if trigger just passed its threshold
     */
    public boolean risingedge(Trigger trigger) {
        boolean returnvalue = false;
        switch (trigger) {
            case TRIGGER_L:
                returnvalue = risingedge(Button.TRIGGER_BUTTON_L);
                break;
            case TRIGGER_R:
                returnvalue = risingedge(Button.TRIGGER_BUTTON_R);
                break;
        }
        return returnvalue;
    }

    /**
     * detect falling edge of a button
     * @param button GamepadEx.Button enumerator representing button to get value of
     * @return true if button was just released (i.e. unpressed now and pressed in previous loop)
     */
    public boolean fallingedge(Button button) {
        boolean returnvalue = false;
        switch (button) {
            case Y:
                returnvalue = !current.y && past.y;
                break;
            case X:
                returnvalue = !current.x && past.x;
                break;
            case A:
                returnvalue = !current.a && past.a;
                break;
            case B:
                returnvalue = !current.b && past.b;
                break;
            case TRIANGLE:
                returnvalue = !current.triangle && past.triangle;
                break;
            case SQUARE:
                returnvalue = !current.square && past.square;
                break;
            case CROSS:
                returnvalue = !current.cross && past.cross;
                break;
            case CIRCLE:
                returnvalue = !current.circle && past.circle;
                break;
            case BACK:
                returnvalue = !current.back && past.back;
                break;
            case START:
                returnvalue = !current.start && past.start;
                break;
            case GUIDE:
                returnvalue = !current.guide && past.guide;
                break;
            case SHARE:
                returnvalue = !current.share && past.share;
                break;
            case OPTIONS:
                returnvalue = !current.options && past.options;
                break;
            case PS:
                returnvalue = !current.ps && past.ps;
                break;
            case BUMPER_L:
                returnvalue = !current.left_bumper && past.left_bumper;
                break;
            case BUMPER_R:
                returnvalue = !current.right_bumper && past.right_bumper;
                break;
            case STICK_L:
                returnvalue = !current.left_stick_button && past.left_stick_button;
                break;
            case STICK_R:
                returnvalue = !current.right_stick_button && past.right_stick_button;
                break;
            case TOUCHPAD_1:
                returnvalue = !current.touchpad_finger_1 && past.touchpad_finger_1;
                break;
            case TOUCHPAD_2:
                returnvalue = !current.touchpad_finger_2 && past.touchpad_finger_2;
                break;
            case DPAD_UP:
                returnvalue = !current.dpad_up && past.dpad_up;
                break;
            case DPAD_DOWN:
                returnvalue = !current.dpad_down && past.dpad_down;
                break;
            case DPAD_LEFT:
                returnvalue = !current.dpad_left && past.dpad_left;
                break;
            case DPAD_RIGHT:
                returnvalue = !current.dpad_right && past.dpad_right;
                break;
            case TRIGGER_BUTTON_L:
                returnvalue = (current.left_trigger < triggerthreshold) && (past.left_trigger >= triggerthreshold);
                break;
            case TRIGGER_BUTTON_R:
                returnvalue = (current.right_trigger < triggerthreshold) && (past.right_trigger >= triggerthreshold);
                break;
        }
        return returnvalue;
    }

    /**
     * detect falling edge of a pair of buttons (both were pressed and at least one was just released)
     * @param button1 GamepadEx.Button enumerator representing button to get value of
     * @param button2 GamepadEx.Button enumerator representing button to get value of
     * @return true if both buttons were down and at least one is currently released
     */
    public boolean fallingedge(Button button1,Button button2) {
        boolean returnvalue = false;
        if (fallingedge(button1)) returnvalue = getpast(button2);
        else if (fallingedge(button2)) returnvalue = getpast(button1);
        return returnvalue;
    }

    /**
     * detect falling edge of a trigger
     * this is identical to the code for treating a trigger as a button
     * this method provided as a convenient alternate way to access that logic
     * @param trigger GamepadEx.Trigger enumerator representing trigger to get value of
     * @return true if trigger just passed below its threshold
     */
    public boolean fallingedge(Trigger trigger) {
        boolean returnvalue = false;
        switch (trigger) {
            case TRIGGER_L:
                returnvalue = fallingedge(Button.TRIGGER_BUTTON_L);
                break;
            case TRIGGER_R:
                returnvalue = fallingedge(Button.TRIGGER_BUTTON_R);
                break;
        }
        return returnvalue;
    }

    /**
     * Rumble the gamepad for a certain number of "blips" using predetermined blip timing
     * This will displace any currently running rumble effect.
     * Note this is a copy of stock Gamepad rumbleBlips that rumbles the left side only
     * (except that it uses the public function runRumbleEffect instead of the private queueeffect)
     * @param count the number of rumble blips to perform
     */
    public void rumbleBlips_2(int count) {
        RumbleEffect.Builder builder = new RumbleEffect.Builder();

        for(int i = 0; i < count; i++) {
            builder.addStep(0,1.0,250).addStep(0,0,100);
        }

        runRumbleEffect(builder.build());
    }


}
