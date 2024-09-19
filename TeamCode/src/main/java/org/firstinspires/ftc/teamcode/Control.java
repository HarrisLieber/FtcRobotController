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

import com.qualcomm.robotcore.util.ElapsedTime;

public class Control {

    private double integral;
    private double lasterror;
    private double kp;
    private double ki;
    private double kd;
    private double int_range;
    // Create a timer for accurately tracking derivative control
    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    // Allow a new controller to be created with no parameters - default to proportional only, kp=1
    public Control() {
        integral = 0;
        lasterror = 0;
        setPIDConstants(1.0, 0,0,0);
    }
    // overload the constructor to allow PID constants to be set
    public Control(double kp, double ki, double kd) {
        integral = 0;
        lasterror = 0;
        setPIDConstants(kp,ki,kd);
    }
    // overload constructor again to allow PID constants and integral range to be set together
    public Control(double kp, double ki, double kd,double ir) {
        integral = 0;
        lasterror = 0;
        setPIDConstants(kp,ki,kd,ir);
    }
    /**
     * Set PID constants kp, ki, and kd
     * @param kp    Proportional constant
     * @param ki    Integral constant
     * @param kd    Derivative constant
     */
    public void setPIDConstants(double kp, double ki, double kd) {
        if (kp >= 0) this.kp = kp;
        if (ki >=0) this.ki = ki;
        if (kd >=0) this.kd = kd;
    }
    /**
     * Set PID constants kp, ki, and kd, and also integral range
     * @param kp    Proportional constant
     * @param ki    Integral constant
     * @param kd    Derivative constant
     * @param ir    Integral range, limiting size of error we allow to be added to integral term
     */
    public void setPIDConstants(double kp, double ki, double kd, double ir) {
        if (kp >= 0) this.kp = kp;
        if (ki >=0) this.ki = ki;
        if (kd >=0) this.kd = kd;
        if (ir >=0) int_range = ir;
    }
    /**
     * Set integral range alone
     * @param ir    Integral range, limiting size of error we allow to be added to integral term
    */
    public void setIntegralRange(double ir) {
        if (ir >= 0) int_range = ir;
    }

    /**
     * Retrieve PID control constants
     * @return array of all four PID constants (kp,ki,kd, and integral range)
     */
    public double[] getPIDConstants() {
        double[] pidarray = {kp,ki,kd,int_range};
        return pidarray;
    }

    public void reset() {
        integral = 0.0d;
        lasterror = 0.0d;
    }

    // Broke out the integral term calculation to improve code reuse
    private double setIntegralTerm(double error) {
        /*
        We add  countermeasures for integral windup since we may have large error at the start of
        control, for example distance control for moving a robot.

        We zero out integral control when we cross the setpoint (i.e. the target) to minimize
        overshoot (integral control is supposed to get us to the target, not make us go past it

        We allow maximum integral range to be set to prevent the integral term from getting
        huge. Outside of that range we limit the integral term to no bigger than error.
        If integral_range=0 we ignore it and just do integral control as normal
        */
        if ((error * lasterror) < 0) {
            // If error and lasterror have different signs, we've crossed the setpoint
            // and zero out the integral term
            integral = 0;
        } else if ((Math.abs(error) > int_range && int_range > 0)) {
            // if error is above the integral range, we don't add it to integral term
            // We limit integral term to no larger than error itself until error is below threshold
            integral = error;
        } else {
            // If neither of the above countermeasures are necessary, we do normal integral control
            // Add error to integral term
            integral += error;
        }
        return integral;
    }
    private double setDerivativeTerm(double error) {
        // calculate derivative term (error - last error)/duration since last loop
        // then reset the timer
        double derivative = (error - lasterror)/timer.milliseconds();
        timer.reset();
        return derivative;
    }
    /**
     * Use PID control to return a feedback term using PID constants
     * PID output = kp * error + ki*integral term + kd * derivative term
     * @param error     error term feeding into the controller
     * @return          PID control output
     */
    public double PID(double error) {
        /*
        PID control roughly is simply output = kp * error + ki*integral term + kd * derivative term
        */
        double pid_interim = kp * error;
        if (ki > 0) pid_interim += setIntegralTerm(error);
        if (kd > 0) pid_interim += setDerivativeTerm(error);
        //save this iteration's error for next iteration
        lasterror = error;
        // return the PID control output
        return pid_interim;
    }

    /**
     * Use PID control to return a feedback term using PID constants
     * PID output = kp * error + ki*integral term + kd * derivative term
     * Note: overloading PID method to allow passing first derivative into function instead of estimating it
     * @param error         error term feeding into the controller
     * @param derivative    derivative term (first derivative i.e. change) feeding into controller
     * @return          PID control output
     */
    public double PID(double error,double derivative) {
        /*
        PID control roughly is simply output = kp * error + ki*integral term + kd * derivative term
        */
        double pid_interim = kp * error;
        if (ki > 0) pid_interim += setIntegralTerm(error);
        if (kd > 0) pid_interim += derivative;
        //save this iteration's error for next iteration
        lasterror = error;
        // return the PID control output
        return pid_interim;

    }

}
