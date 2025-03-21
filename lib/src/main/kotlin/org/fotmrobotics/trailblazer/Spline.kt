/*
Copyright (c) 2024, Fire on the Mountain Robotics
All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree.
*/

package org.fotmrobotics.trailblazer

import kotlin.math.min
import kotlin.math.max

class Spline() {
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

    fun getPt(i: Int): Vector2D {
        return controlPoints[i]
    }

    fun getLength(): Int {
        return this.length
    }

    fun setSegment(i: Int) {this.currentSegment = i}

    fun incSegment() {
        if (this.currentSegment < length - 4) {this.currentSegment++}
    }

    fun decSegment() {
        if (this.currentSegment > 0) {this.currentSegment--}
    }

    fun getSegment(): Int {
        return this.currentSegment
    }

    fun getSegmentPoints(): ArrayList<Vector2D> {
        val points = ArrayList<Vector2D>()
        val segment = max(min(this.currentSegment, length - 4), 0)
        for (i in 0..3) {points.add(controlPoints[segment + i])}
        return points
    }

    fun getSegmentPoints(segment: Int): ArrayList<Vector2D> {
        val points = ArrayList<Vector2D>()
        val targetSegment = max(min(segment, length - 4), 0)
        for (i in 0..3) {points.add(controlPoints[targetSegment + i])}
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
        return catmullRomSpline(t % 1, segmentPoints[0], segmentPoints[1], segmentPoints[2], segmentPoints[3])
    }

    fun getPoint(segment: Int, t: Double): Vector2D {
        val segmentPoints = getSegmentPoints(segment)
        return catmullRomSpline(t, segmentPoints[0], segmentPoints[1], segmentPoints[2], segmentPoints[3])
    }

    fun getDeriv(t: Double): Vector2D {
        val segmentPoints = getSegmentPoints()
        return catmullRomSplineDerivative(t, segmentPoints[0], segmentPoints[1], segmentPoints[2], segmentPoints[3])
    }

    fun getDeriv(segment: Int, t: Double): Vector2D {
        val segmentPoints = getSegmentPoints(segment)
        return catmullRomSplineDerivative(t, segmentPoints[0], segmentPoints[1], segmentPoints[2], segmentPoints[3])
    }

    fun getDeriv2(t: Double): Vector2D {
        val segmentPoints = getSegmentPoints()
        return catmullRomSplineDerivative2(t, segmentPoints[0], segmentPoints[1], segmentPoints[2], segmentPoints[3])
    }
}