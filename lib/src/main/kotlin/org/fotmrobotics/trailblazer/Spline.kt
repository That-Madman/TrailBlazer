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
    = (d1.x * d2.y - d1.y * d2.x) / d1.norm().pow(3)

fun distance (p1: Vector2D, p0: Vector2D)
    = (p1 - p0).norm().pow(2)

fun distanceDeriv (p1: Vector2D, p0: Vector2D, dp1: Vector2D)
    = ((p1 - p0) * 2.0) dot dp1

// Use this
fun distanceDeriv2 (p1: Vector2D, p0: Vector2D, dp1: Vector2D, ddp1: Vector2D)
    = ((((p1 - p0) * ddp1) + (dp1 * dp1)) * 2.0).norm()

fun distanceDeriv2 (t: Double, p0: Vector2D, p1: Vector2D, p2: Vector2D, p3: Vector2D, pos: Vector2D): Double {
    val h = 1e-4
    return (distanceDeriv(slerp(t+h, p0, p1, p2, p3), pos, slerpDeriv(t+h, p0, p1, p2 ,p3)) -
            distanceDeriv(slerp(t, p0, p1, p2, p3), pos, slerpDeriv(t, p0, p1, p2, p3))) / h
}

fun closestPoint (pos: Vector2D, p0: Vector2D, p1: Vector2D, p2: Vector2D, p3: Vector2D, maxIteration: Int = 1000, tolerance: Double = 1e-6): Double {
    var t = 0.5

    for (i in 1..maxIteration) {
        val splinePoint = slerp(t, p0, p1, p2, p3)
        val splineDeriv = slerpDeriv(t, p0, p1, p2, p3)
        val splineDeriv2 = slerpDeriv2(t, p0, p1, p2, p3)

        val df = distanceDeriv(splinePoint, pos, splineDeriv)
        val ddf = distanceDeriv2(splinePoint, pos, splineDeriv, splineDeriv2)

        t -= df / ddf
        t = min(max(t, 0.0), 1.0)

        if (splinePoint == pos) {break}
        if (abs(df) < tolerance) {break}
    }

    return t
}

class Spline() {
    // TODO: If use hermite add new list for tangents. Every point must have a corresponding
    //  tangent, if not the tangent can be calculated using catmull-rom spline
    private lateinit var controlPoints: ArrayList<Vector2D>
    private var segment = 0
    private var length = 0

    constructor(controlPoints: ArrayList<Vector2D>): this() {
        this.controlPoints = controlPoints
        this.length = controlPoints.size
    }

    fun add(point: Vector2D) {
        controlPoints.add(point)
        length++
    }

    fun add(i: Int, point: Vector2D) {
        controlPoints.add(i, point)
        length++
    }

    fun set(i: Int, point: Vector2D) {
        controlPoints[i] = point
    }

    fun remove(i: Int) {
        controlPoints.removeAt(i)
        length--
    }

    fun setSegment(i: Int) {segment = i}

    fun incSegment() {
        if (segment < length-1) {segment++}
    }

    fun decSegment() {
        if (segment > 0) {segment--}
    }

    fun getSegment(): Int {
        return segment
    }

    fun getSegmentPoints(): ArrayList<Vector2D> {
        val points = ArrayList<Vector2D>()
        for (i in 0..3) {points.add(controlPoints[segment+i])}
        return points
    }

    fun getClosest(pos: Vector2D): Double {
        val segmentPoints = getSegmentPoints()

        var t: Double
        while (true) {
            t = closestPoint(
                pos,
                segmentPoints[0],
                segmentPoints[1],
                segmentPoints[2],
                segmentPoints[3]
            )

            when {
                (t in 0.0..1.0) -> break
                (segment == 0 || segment == length-1) -> break
                (t < 0) -> segment--
                (t > 1) -> segment++
            }
        }

        return t
    }

    fun getPoint(t: Double): Vector2D {
        val segmentPoints = getSegmentPoints()
        return slerp(t, segmentPoints[0], segmentPoints[1], segmentPoints[2], segmentPoints[3])
    }

    fun getDeriv(t: Double): Vector2D {
        val segmentPoints = getSegmentPoints()
        return slerpDeriv(t, segmentPoints[0], segmentPoints[1], segmentPoints[2], segmentPoints[3])
    }

    fun getDeriv2(t: Double): Vector2D {
        val segmentPoints = getSegmentPoints()
        return slerpDeriv2(t, segmentPoints[0], segmentPoints[1], segmentPoints[2], segmentPoints[3])
    }
}

/*
class Spline(var controlPoints: ArrayList<Vector2D>) {
    private var n: Int = controlPoints.size
    private var segment: Int

    init {
        if (n == 0) {
            error("Spline must be initialized with 2 or more control points.")
        }

        controlPoints.add(controlPoints.last())
        controlPoints.add(0, controlPoints.first())
        segment = 1
    }

    fun addPoint (i: Int, point: Vector2D) {
        controlPoints.add(i, point)
        n++
    }

    fun getSegment (): ArrayList<Vector2D> {
        val points = ArrayList<Vector2D>()
        for (i in 0..3) {points.add(controlPoints[segment-1+i])}
        return points
    }

    fun setSegment (i: Int) {segment = i}

    fun numSegments (): Int = n

    fun incSegment () {
        if (segment < n) {segment++}
    }

    fun decSegment () {
        if (segment > 1) {segment--}
    }
    //

    fun getClosestPoint (pos: Vector2D): Double {
        val segmentPoints = getSegment()
        return closestPoint(pos, segmentPoints[0], segmentPoints[1], segmentPoints[2], segmentPoints[3])
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
}*/