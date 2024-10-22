/*
Copyright (c) 2024, Fire on the Mountain Robotics
All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree.
*/

package org.fotmrobotics.trailblazer

fun angleWrap (angle: Double) =
    if (angle >= 0) {angle % 360}
    else {angle % 360 + 360}