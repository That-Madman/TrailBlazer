/*
 * Copyright (c) 2024, Fire on the Mountain Robotics
 * All rights reserved.
 *
 * This source code is licensed under the BSD-style license found in the
 * LICENSE file in the root directory of this source tree.
*/

package org.fotmrobotics.trailblazer

import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.pow

fun angleWrap (angle: Double) =
    if (angle >= 0) {angle % 360}
    else {angle % 360 + 360}

// Catmull-rom spline interpolation
fun catmullRomSpline (t: Double, p0: Vector2D, p1: Vector2D, p2: Vector2D, p3: Vector2D)
    = (
        (p1*2.0) +
        (-p0 + p2) * t +
        (p0*2.0 - p1*5.0 + p2*4.0 - p3) * t*t +
        (-p0 + p1*3.0 - p2*3.0 + p3) * t*t*t
    ) * 0.5

// Derivative of catmull-rom spline interpolation
fun catmullRomSplineDerivative (t: Double, p0: Vector2D, p1: Vector2D, p2: Vector2D, p3: Vector2D)
    = (
        (p1*3.0 + p3 - p0 - p2*3.0) * 3.0*t*t +
        (p0*2.0 + p2*4.0 - p1*5.0 - p3) * 2.0*t +
        (-p0 + p2)
    ) * 0.5

// 2nd derivative of catmull-rom spline interpolation
fun catmullRomSplineDerivative2 (t: Double, p0: Vector2D, p1: Vector2D, p2: Vector2D, p3: Vector2D)
    = (p1*3.0 + p3 - p0)*3.0*t + (p2*5.0 - p1*5.0 + p0 - p3)*2.0

// Curvature at a given point
fun curvature (spline: Spline, t: Double, distance: Double = 1e-4): Double {
    val p0 = spline.getPoint(t - distance)
    val p1 = spline.getPoint(t)
    val p2 = spline.getPoint(t + distance)

    return (p0.x * (p1.y - p2.y) +
            p1.x * (p2.y - p0.y) +
            p2.x * (p0.y - p1.y)) /
            (
                    0.125 * Math.pow(
                        Math.pow(p2.x - p0.x, 2.0) +
                                Math.pow(p2.y - p0.y, 2.0)
                        , 3.0 / 2.0)
                    )
}

fun distance (p1: Vector2D, p0: Vector2D)
    = (p1 - p0).norm().pow(2)

fun distanceDerivative (p1: Vector2D, p0: Vector2D, dp1: Vector2D)
    = ((p1 - p0) * 2.0) dot dp1

fun distanceDerivative2 (p1: Vector2D, p0: Vector2D, dp1: Vector2D, ddp1: Vector2D)
    = ((((p1 - p0) * ddp1) + (dp1 * dp1)) * 2.0).norm()

fun closestPoint (pos: Vector2D, p0: Vector2D, p1: Vector2D, p2: Vector2D, p3: Vector2D, maxIteration: Int = 1000, tolerance: Double = 1e-6): Double {
    var t = 0.5

    for (i in 1..maxIteration) {
        val splinePoint = catmullRomSpline(t, p0, p1, p2, p3)
        val splineDeriv = catmullRomSplineDerivative(t, p0, p1, p2, p3)
        val splineDeriv2 = catmullRomSplineDerivative2(t, p0, p1, p2, p3)

        val df = distanceDerivative(splinePoint, pos, splineDeriv)
        val ddf = distanceDerivative2(splinePoint, pos, splineDeriv, splineDeriv2)

        t -= df / ddf
        t = min(max(t, 0.0), 1.0)

        if (splinePoint == pos) {break}
        if (abs(df) < tolerance) {break}
    }

    return t
}
