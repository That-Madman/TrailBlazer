/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2023, Alex Bryan
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

package org.fotmrobotics.trailblazer

/**
 * A PID class originally written for fun, it was well proven for use during the 2023-2024 season
 * of FTC. It uses the traditional PID controller model, with the addition of a feedforward value,
 * which is a common addition for more complex control. For more advanced control, it allows for
 * functions or lambdas to also be passed, allowing for a more streamlined control for later use or
 * overriding certain aspects of the code if used in an abnormal environment
 * @param kP the P value
 * @param kI the value for the magnitude of change in the [I][i] value
 * @param kD the D value
 * @param kF the F value
 * @param posGet the function to get the current value of what is being influenced
 * @param exFun the function executed when pidCalc is called
 * @param timeGet the function to get the time
 * @property update calculates the PID value and executes exFun if given
 * @property pidCalc same as [update][update], with the addition that it calculates the error for
 * you, allowing you to utilize the [position function][posGet].
 * @property resetI resets the [I][i] value that gets accrued as the program runs
 *
 * @author Alex Bryan
 */
class PIDF @JvmOverloads constructor(
    var kP: Double,
    var kI: Double,
    var kD: Double,
    var kF: Double = 0.0,
    var posGet: (() -> Number)? = null,
    var exFun: ((Number) -> Unit)? = null,
    var timeGet: (() -> Number)? = { System.nanoTime() / 1e9 },
) {
    /**
     * In PID, the I value is a value that gets aggregated while the formula goes on. It increases
     * based off the difference between the current and target position, multiplied by [kI]. It is
     * used to address a constant error, slowly increasing or decreasing to help sustain the target
     * position. If [kP] is 0, i will never change, which is sometimes the best option. If
     * you want to reset it back to 0, use the [resetI] function
     */
    private var i: Double = 0.0
    private var maxI: Double = Double.NaN

    private var prevTime = 0.0
    private var prevErr = 0.0

    /**
     * Calculates the output for PID based off of the [P][kP], [I][i], and [D][kD] values.
     * If the [execute function][exFun] was set, the function also executes it. It is not used in
     * the quickstart, though it is still kept if the [update function][update] doesn't cut it.
     * @param target The target value
     * @param currPos The current value. If it is not set, the function will instead use the
     * [value obtaining function][posGet].
     * @param time The current time. If it is not set, the function will instead use the
     * [time function][timeGet].
     * @return the calculated PID output, to be plugged into a function if the [execute function]
     * [exFun] wasn't already set.
     * @author Alex Bryan
     */
    @JvmOverloads
    fun pidCalc(
        target: Number,
        currPos: Number = if (posGet != null) posGet!!.invoke() else 0,
        time: Number = if (timeGet != null) timeGet!!.invoke() else 0
    ): Double {
        val currErr: Double = target.toDouble() - currPos.toDouble()
        val p = kP * currErr

        i += kI * (currErr * (time.toDouble() - prevTime))

        if (!maxI.isNaN()) i = i.coerceIn(-maxI..maxI)

        val d = kD * (currErr - prevErr) / (time.toDouble() - prevTime)
        prevErr = currErr
        prevTime = time.toDouble()

        if (exFun != null) exFun!!.invoke(p + i + d)
        return p + i + d + kF
    }

    /**
     * Calculates the output for PID based off of the [P][kP], [I][i], and [D][kD] values.
     * If the [execute function][exFun] was set, the function also executes it.
     * @param currErr The current error.
     * @param time The current time. If it is not set, the function will instead use the
     * [time function][timeGet].
     * @return the calculated PID output, to be plugged into a function if the [execute function]
     * [exFun] wasn't already set.
     * @author Alex Bryan
     */
    @JvmOverloads
    fun update (
        currErr: Number, time: Number = if (timeGet != null) timeGet!!.invoke() else 0
    ): Double {
        val p = kP * currErr.toDouble()

        i += kI * (currErr.toDouble() * (time.toDouble() - prevTime))

        if (!maxI.isNaN()) i = i.coerceIn(-maxI..maxI)

        val d = kD * (currErr.toDouble() - prevErr) / (time.toDouble() - prevTime)
        prevErr = currErr.toDouble()
        prevTime = time.toDouble()

        if (exFun != null) exFun!!.invoke(p + i + d)
        return p + i + d + kF
    }

    /**
     * Resets the [I][i] value back to 0
     * @author Alex Bryan
     */
    fun resetI() {
        i = 0.0
    }
}