/*
Copyright (c) 2024, Fire on the Mountain Robotics
All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree.
*/

package org.fotmrobotics.trailblazer

fun driveVector (spline: Spline, pos: Pose2D, pidf: PIDF): Pose2D {
    val t = spline.getClosestPoint(pos)

    val splinePoint = spline.getPoint(t)
    val splineDeriv = spline.getDeriv(t)
    val splineDeriv2 = spline.getDeriv2(t)

    val distance = splinePoint - pos
    val output = pidf.update(distance.norm())
    val translation = distance * output

    val forward = splineDeriv

    val curvature = curvature(splineDeriv, splineDeriv2)
    val centripetal = forward.pow(2) * curvature

    val drive = forward + translation + centripetal

    return Pose2D(drive.x, drive.y, 0.0)
}

fun drivePowers () {}