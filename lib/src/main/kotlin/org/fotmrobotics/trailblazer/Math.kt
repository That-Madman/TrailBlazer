package org.fotmrobotics.trailblazer

fun angleWrap (angle: Double) =
    if (angle >= 0) {angle % 360}
    else {angle % 360 + 360}