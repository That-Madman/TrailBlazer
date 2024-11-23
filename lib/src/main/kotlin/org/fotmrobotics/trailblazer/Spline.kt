/*
Copyright (c) 2024, Fire on the Mountain Robotics
All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree.
*/

package org.fotmrobotics.trailblazer

import org.apache.commons.math3.geometry.euclidean.twod.Segment
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
fun curvature (spline: Spline, t: Double, distance: Double = 1e-4): Double {
    val p0 = spline.getPoint(t - distance)
    val p1 = spline.getPoint(t)
    val p2 = spline.getPoint(t + distance)

    return (p0.x * (p1.y - p2.y) +
            p1.x * (p2.y - p0.y) +
            p2.x + (p0.y - p1.y)) /
            (
                0.125 * Math.pow(
                    Math.pow(p2.x - p0.x, 2.0) +
                    Math.pow(p2.y - p0.y, 2.0)
                    , 3.0 / 2.0)
            )
}

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
    private var currentSegment = 0
    private var length = 0

    constructor(controlPoints: ArrayList<Vector2D>): this() {
        this.controlPoints = controlPoints
        this.length = controlPoints.size
    }

    fun addPt(point: Vector2D) {
        controlPoints.add(point)
        length++
    }

    fun addPt(i: Int, point: Vector2D) {
        controlPoints.add(i, point)
        length++
    }

    fun setPt(i: Int, point: Vector2D) {
        controlPoints[i] = point
    }

    fun removePt(i: Int) {
        controlPoints.removeAt(i)
        length--
    }

    fun setSegment(i: Int) {this.currentSegment = i}

    fun incSegment() {
        if (this.currentSegment < length-1) {this.currentSegment++}
    }

    fun decSegment() {
        if (this.currentSegment > 0) {this.currentSegment--}
    }

    fun getSegment(): Int {
        return this.currentSegment
    }

    fun getSegmentPoints(): ArrayList<Vector2D> {
        val points = ArrayList<Vector2D>()
        for (i in 0..3) {points.add(controlPoints[this.currentSegment + i])}
        return points
    }

    fun getSegmentPoints(segment: Int): ArrayList<Vector2D> {
        val points = ArrayList<Vector2D>()
        val targetSegment = max(min(segment, length - 1), 0)
        for (i in 0..3) {points.add(controlPoints[targetSegment+i])}
        return points
    }

    fun getClosestPoint(pos: Vector2D): Double {
        val segmentPoints = getSegmentPoints()

        val t = closestPoint(
            pos,
            segmentPoints[0],
            segmentPoints[1],
            segmentPoints[2],
            segmentPoints[3]
        )

        return t
    }

    fun getPoint(t: Double): Vector2D {
        val i = t.toInt()
        val segmentPoints = getSegmentPoints(this.currentSegment + i)
        return slerp(t % 1, segmentPoints[0], segmentPoints[1], segmentPoints[2], segmentPoints[3])
    }

    fun getPoint(segment: Int, t: Double): Vector2D {
        val segmentPoints = getSegmentPoints(segment)
        return slerp(t, segmentPoints[0], segmentPoints[1], segmentPoints[2], segmentPoints[3])
    }

    fun getDeriv(t: Double): Vector2D {
        val segmentPoints = getSegmentPoints()
        return slerpDeriv(t, segmentPoints[0], segmentPoints[1], segmentPoints[2], segmentPoints[3])
    }

    fun getDeriv(segment: Int, t: Double): Vector2D {
        val segmentPoints = getSegmentPoints(segment)
        return slerpDeriv(t, segmentPoints[0], segmentPoints[1], segmentPoints[2], segmentPoints[3])
    }

    // May delete if not needed since 2nd deriv is not continuous
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