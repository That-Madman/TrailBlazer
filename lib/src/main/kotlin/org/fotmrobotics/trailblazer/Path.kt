/*
Copyright (c) 2024, Fire on the Mountain Robotics
All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree.
*/

package org.fotmrobotics.trailblazer

import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.sin

fun driveVector (spline: Spline, pos: Pose2D, pidf: PIDF): Pose2D {
    val t = spline.getClosestPoint(pos)

    val splinePoint = spline.getPoint(t)
    val splineDeriv = spline.getDeriv(t)

    var angle = atan2(splineDeriv.y, splineDeriv.x) % (2 * Math.PI)
    //if (splineDeriv.x < 0) angle += Math.PI
    //var angle = atan2(splineDeriv.y, splineDeriv.x)
    //val splineDeriv2 = spline.getDeriv2(t)

    val distance = splinePoint - pos
    val output = pidf.update(distance.norm())
    val translation = distance * output

    val forward = splineDeriv

    val k = curvature(spline, t)
    val centripetalMagnitude = forward.norm().pow(2.0) * k
    val centripetal = Vector2D(
        centripetalMagnitude * cos(angle + Math.PI / 2),
        centripetalMagnitude * sin(angle + Math.PI / 2)
        )

    val drive = forward + translation + centripetal

    return Pose2D(drive.x, drive.y, angle)
}

fun test(spline: Spline, pos: Pose2D, pidf: PIDF): Pose2D {
    val t = spline.getClosestPoint(pos)
    val splineDeriv = spline.getDeriv(t)
    var angle = atan2(splineDeriv.y, splineDeriv.x) % (2 * Math.PI)
    if (splineDeriv.x < 0) angle += Math.PI
    val forward = splineDeriv
    val k = curvature(spline, t)
    val centripetalMagnitude = forward.norm().pow(2.0) * k
    val centripetal = Vector2D(
        centripetalMagnitude * cos(angle + Math.PI / 2),
        centripetalMagnitude * sin(angle + Math.PI / 2)
    )

    return Pose2D(centripetal.x, centripetal.y, k)
}

/*class PathSegment () {
    enum class Heading {
        FOLLOW, CONSTANT, OFFSET
    }
}*/

class Event(type: Type, pt: Vector2D, d: Double, value: Any? = null) {
    val grid = Vector2D(pt.x % 12, pt.y % 12) // Not needed, remove


    enum class Type {
        ACTION,
        HEADING,
        STOP
    }

    enum class Heading {
        FOLLOW,
        CONSTANT,
        OFFSET
    }

    //constructor(type: Event.Type, pt: Vector2D, d: Double, value: Any, queue: Boolean) : this(type, pt, d, value) {
        //val queue = true
    //}
}