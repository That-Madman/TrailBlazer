/*
 * Copyright (c) 2024, Fire on the Mountain Robotics
 * All rights reserved.
 *
 * This source code is licensed under the BSD-style license found in the
 * LICENSE file in the root directory of this source tree.
*/

package org.fotmrobotics.trailblazer

import kotlin.math.sqrt
import kotlin.math.atan2
import kotlin.math.PI
import kotlin.math.pow

open class Vector2D (var x: Double, var y: Double) {
    operator fun plus (v: Vector2D) = Vector2D(x + v.x, y + v.y)

    operator fun inc() = Vector2D(x + 1, y + 1)

    operator fun minus (v: Vector2D) = Vector2D(x - v.x, y - v.y)

    operator fun dec() = Vector2D(x - 1, y - 1)

    operator fun times (value: Double) = Vector2D(x * value, y * value)

    operator fun times (v: Vector2D) = Vector2D(x * v.x, y * v.y)

    operator fun div (value: Double) = Vector2D(x / value, y / value)

    operator fun rem (value: Double) = Vector2D(x % value, y % value)

    open operator fun get (i: Int): Double {
        return when (i) {
            0 -> x
            1 -> y
            else -> error("Index out of range.")
        }
    }

    open operator fun set (i: Int, value: Double) {
        when (i) {
            0 -> {x = value}
            1 -> {y = value}
            else -> error("Index out of range.")
        }
    }

    fun set (x: Double, y: Double) {
        this.x = x
        this.y = y
    }


    fun set (vector2d: Vector2D) {
        this.x = vector2d.x
        this.y = vector2d.y
    }

    operator fun compareTo (v: Vector2D): Int {
        return when {
            this.norm() > v.norm() -> 1
            this.norm() < v.norm() -> -1
            else -> 0
        }
    }

    infix fun dot (v: Vector2D) = x * v.x + y * v.y

    infix fun x (v: Vector2D) = x*v.y - y*v.x

    infix fun proj (v: Vector2D) = v * (this dot v) / (v dot v)

    operator fun unaryMinus() = Vector2D(-x, -y)

    fun pow (n: Int) = Vector2D(x.pow(n), y.pow(n))

    fun sum() = x + y

    fun norm() = sqrt(x*x + y*y)

    fun angle() = angleWrap(atan2(y, x) * 180 / PI)

    override fun toString(): String {
        return "{$x, $y}"
    }
}

class Pose2D (x: Double, y: Double, var h: Double): Vector2D(x, y) {
    override operator fun get (i: Int): Double {
        return when (i) {
            0 -> x
            1 -> y
            2 -> h
            else -> error("Index out of range.")
        }
    }

    override operator fun set (i: Int, value: Double) {
        when (i) {
            0 -> {x = value}
            1 -> {y = value}
            2 -> {h = value}
            else -> error("Index out of range.")
        }
    }

    fun set (x: Double, y: Double, h: Double) {
        this.x = x
        this.y = y
        this.h = h
    }

    fun set (pose2d: Pose2D) {
        this.x = pose2d.x
        this.y = pose2d.y
        this.h = pose2d.h
    }

    override fun toString(): String {
        return "${super.toString()} facing $h"
    }
}