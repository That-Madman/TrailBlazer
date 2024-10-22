/*
Copyright (c) 2024, Fire on the Mountain Robotics
All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree.
*/

package org.fotmrobotics.trailblazer

import kotlin.math.pow
import kotlin.math.min
import kotlin.math.max
import kotlin.math.abs

// Catmull-rom spline interpolation
fun slerp (t: Double, p0: Vector2D, p1: Vector2D, p2: Vector2D, p3: Vector2D)
    = ((p1*2.0)+(-p0+p2)*t+(p0*2.0-p1*5.0+p2*4.0-p3)*t*t+(-p0+p1*3.0-p2*3.0+p3)*t*t*t)*0.5

// Derivative of catmull-rom spline interpolation
fun slerpDeriv (t: Double, p0: Vector2D, p1: Vector2D, p2: Vector2D, p3: Vector2D)
    = ((p1*3.0+p3-p0-p2*3.0)*3.0*t*t+(p0*2.0+p2*4.0-p1*5.0-p3)*2.0*t+(-p0+p2))*0.5

// 2nd derivative of catmull-rom spline interpolation
fun slerpDeriv2 (t: Double, p0: Vector2D, p1: Vector2D, p2: Vector2D, p3: Vector2D)
    = (p1*3.0+p3-p0)*3.0*t+(p2*5.0-p1*5.0+p0-p3)*2.0

// Curvature at a given point
fun curvature (d1: Vector2D, d2: Vector2D)
    = (d1 x d2) / d1.norm().pow(3)

fun distance (p1: Vector2D, p0: Vector2D)
    = (p1 - p0).norm().pow(2)

fun distanceDeriv (p1: Vector2D, p0: Vector2D, dp1: Vector2D)
    = ((p1 - p0) * 2.0) dot dp1

//fun distanceDeriv2 (p1: Vector2D, p0: Vector2D, dp1: Vector2D, ddp1: Vector2D)
    //= 2 * ((ddp1 dot (p1 - p0)) + (dp1 dot dp1))

fun distanceDeriv2 (t: Double, p0: Vector2D, p1: Vector2D, p2: Vector2D, p3: Vector2D, pos: Vector2D): Double {
    val h = 1e-4
    return (distanceDeriv(slerp(t+h, p0, p1, p2, p3), pos, slerpDeriv(t+h, p0, p1, p2 ,p3)) -
            distanceDeriv(slerp(t, p0, p1, p2, p3), pos, slerpDeriv(t, p0, p1, p2, p3))) / h
}

fun closestPoint (pos: Vector2D, p0: Vector2D, p1: Vector2D, p2: Vector2D, p3: Vector2D, maxIteration: Int = 1000, tolerance: Double = 1e-6): Double {
    //val maxIteration = 1000
    //val tolerance = 1e-6
    var t = 0.5

    var splinePoint = Vector2D(Double.NaN, Double.NaN)
    var splineDeriv = Vector2D(Double.NaN, Double.NaN)
    var splineDeriv2 = Vector2D(Double.NaN, Double.NaN)

    for (i in 1..maxIteration) {
        splinePoint = slerp(t, p0, p1, p2, p3)
        splineDeriv = slerpDeriv(t, p0, p1, p2, p3)
        splineDeriv2 = slerpDeriv2(t, p0, p1, p2, p3)

        val df = distanceDeriv(splinePoint, pos, splineDeriv)
        val ddf = distanceDeriv2(t, p0, p1, p2, p3, pos)

        t -= df / ddf
        t = min(max(t, 0.0), 1.0)

        if (splinePoint == pos) {break}
        if (abs(df) < tolerance) {break}
    }

    return t
}

fun closestPointTest (pos: Vector2D, p0: Vector2D, p1: Vector2D, p2: Vector2D, p3: Vector2D): Double {
    val maxIteration = 1000
    val tolerance = 1e-8
    var t = 0.5

    var splinePoint = Vector2D(Double.NaN, Double.NaN)
    var splineDeriv = Vector2D(Double.NaN, Double.NaN)
    var splineDeriv2 = Vector2D(Double.NaN, Double.NaN)

    splinePoint = slerp(t, p0, p1, p2, p3)
    splineDeriv = slerpDeriv(t, p0, p1, p2, p3)
    splineDeriv2 = slerpDeriv2(t, p0, p1, p2, p3)
    val df = distanceDeriv(splinePoint, pos, splineDeriv)
    //val ddf = distanceDeriv2(splinePoint, pos, splineDeriv, splineDeriv2)
    val ddf = distanceDeriv2(t, p0, p1, p2, p3, pos)
    /*
    for (i in 1..maxIteration) {
        splinePoint = slerp(t, p0, p1, p2, p3)
        splineDeriv = slerpDeriv(t, p0, p1, p2, p3)
        splineDeriv2 = slerpDeriv2(t, p0, p1, p2, p3)

        //if (splinePoint == pos) {break}

        df = distanceDeriv(splinePoint, pos, splineDeriv)
        val ddf = distanceDeriv2(splinePoint, pos, splineDeriv, splineDeriv2)

        //if (abs(df) < tolerance) {break}

        //t -= df / ddf
        //t = min(max(t, 0.0), 1.0)
    }
     */

    return ddf
}

// TODO: Change to newton's method
/*fun gradientDescent (p0: Vector2D, p1: Vector2D, p2: Vector2D, p3: Vector2D, pos: Vector2D): Triple<Double, Vector2D, Vector2D> {
    val learningRate = 0.01
    val maxIteration = 1000
    val tolerance = 1e-6

    var t = 0.5

    var splinePoint = Vector2D(Double.NaN, Double.NaN)
    var splineDeriv = Vector2D(Double.NaN, Double.NaN)

    for (i in 1..maxIteration) {
        splinePoint = slerp(t, p0, p1, p2, p3)
        splineDeriv = slerpDeriv(t, p0, p1, p2, p3)

        if (splinePoint == pos) {break}

        val distanceGradient = splinePoint - pos
        val gradient = (distanceGradient dot splineDeriv) / distanceGradient.norm()

        val last_t = t

        t -= learningRate * gradient

        t = min(max(t,0.0), 1.0)

        if (abs(t - last_t) < tolerance) {break}
    }

    return Triple(t, splinePoint, splineDeriv)
}*/

class Spline(var controlPoints: ArrayList<Vector2D>) {
    private val n: Int = controlPoints.size - 1
    private var segment: Int

    init {
        if (n == 0) {
            error("Spline must be initialized with 2 or more control points.")
        }

        controlPoints.add(controlPoints.last())
        controlPoints.add(0, controlPoints.first())
        segment = 1
    }

    fun getSegment (): ArrayList<Vector2D> {
        val points = ArrayList<Vector2D>()
        for (i in 0..3) {points.add(controlPoints[segment-1+i])}
        return points
    }

    fun setSegment (x: Int) {segment = x}

    fun numSegments (): Int = n

    // TODO: Maybe change to operator?
    fun incSegment () {
        if (segment == n) {segment = 1}
        else {segment++}
    }

    fun decSegment () {
        if (segment == 1) {segment = n}
        else {segment--}
    }
    //

    fun getClosestPoint (pos: Vector2D): Double {
        val segmentPoints = getSegment()
        return closestPoint(pos, segmentPoints[0], segmentPoints[1], segmentPoints[2], segmentPoints[3])
    }

    fun getClosestPointTest (pos: Vector2D): Double {
        val segmentPoints = getSegment()
        return closestPointTest(pos, segmentPoints[0], segmentPoints[1], segmentPoints[2], segmentPoints[3])
    }

    fun getPoint (t: Double): Vector2D {
        val segmentPoints = getSegment()
        return slerp(t, segmentPoints[0], segmentPoints[1], segmentPoints[2], segmentPoints[3])
    }

    fun getDeriv (t: Double): Vector2D {
        val segmentPoints = getSegment()
        return slerpDeriv(t, segmentPoints[0], segmentPoints[1], segmentPoints[2], segmentPoints[3])
    }

    fun getDeriv2 (t: Double): Vector2D {
        val segmentPoints = getSegment()
        return slerpDeriv2(t, segmentPoints[0], segmentPoints[1], segmentPoints[2], segmentPoints[3])
    }
}