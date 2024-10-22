/*
Copyright (c) 2024, Fire on the Mountain Robotics
All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree.
*/

package org.fotmrobotics.trailblazer

class PIDF (private var kP: Double, private var kI: Double, private var kD: Double, private var kF: Double) {
    operator fun set(i: Int, value: Double) {
        when (i) {
            0 -> {kP = value}
            1 -> {kI = value}
            2 -> {kD = value}
            3 -> {kF = value}
            else -> error("Index out of range.")
        }
    }

    private var prevTime = 0.0

    private fun getTimeDifference(): Double {
        val dt = (System.currentTimeMillis() / 1000.0) - prevTime;
        prevTime = System.currentTimeMillis() / 1000.0
        return dt
    }

    private var prevError = 0.0

    fun update (error: Double): Double {
        val dt = getTimeDifference()
        prevError = error
        return p(error) + i(error, dt) + d(error, dt) + f()
    }

    private fun p (error: Double): Double {
        return kP * error
    }

    private var integralSum = 0.0

    private fun i (error: Double, dt: Double): Double {
        integralSum += error * dt
        return kI * integralSum
    }

    private fun d (error: Double, dt: Double): Double {
        val derivative = (error - prevError) / dt
        return kD * derivative
    }

    private fun f (): Double {
        return kF
    }
}