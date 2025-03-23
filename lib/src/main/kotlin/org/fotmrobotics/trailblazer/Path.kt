/*
 * Copyright (c) 2024, Fire on the Mountain Robotics
 * All rights reserved.
 *
 * This source code is licensed under the BSD-style license found in the
 * LICENSE file in the root directory of this source tree.
*/

package org.fotmrobotics.trailblazer

import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.min
import kotlin.math.pow
import kotlin.math.sin

fun driveVector (spline: Spline, pos: Pose2D, pidf: PIDF): Pose2D {
    val t = spline.getClosestPoint(pos)

    val splinePoint = spline.getPoint(t)
    val splineDeriv = spline.getDeriv(t)

    var angle = atan2(splineDeriv.y, splineDeriv.x)
    //if (splineDeriv.x < 0) angle += Math.PI
    //var angle = atan2(splineDeriv.y, splineDeriv.x)
    //val splineDeriv2 = spline.getDeriv2(t)

    val distance = splinePoint - pos
    val output = pidf.update(distance.norm())
    val translation = distance * output

    val forward = splineDeriv

    val k = curvature(spline, t)
    val centripetalMagnitude = min(forward.norm().pow(2.0) * k, 1.0)
    val centripetal = Vector2D(
        centripetalMagnitude * cos(angle + Math.PI / 2),
        centripetalMagnitude * sin(angle + Math.PI / 2)
        )

    val drive = forward + translation //+ centripetal

    return Pose2D(drive.x, drive.y, angle)
}