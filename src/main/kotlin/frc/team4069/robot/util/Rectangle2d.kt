package frc.team4069.robot.util

import edu.wpi.first.wpilibj.geometry.Translation2d
import frc.team4069.saturn.lib.mathematics.twodim.geometry.Translation2d
import frc.team4069.saturn.lib.mathematics.units.Meter
import frc.team4069.saturn.lib.mathematics.units.SIUnit
import frc.team4069.saturn.lib.mathematics.units.meter
import kotlin.math.max
import kotlin.math.min

data class Rectangle2d(
    val x: SIUnit<Meter>,
    val y: SIUnit<Meter>,
    val w: SIUnit<Meter>,
    val h: SIUnit<Meter>
) {

    val topLeft = Translation2d(x, y + h)
    val topRight = Translation2d(x + w, y + h)
    val bottomLeft = Translation2d(x, y)
    val bottomRight = Translation2d(x + w, y)

    val maxCorner = topRight
    val minCorner = bottomLeft

    constructor(xRange: ClosedRange<SIUnit<Meter>>,
                yRange: ClosedRange<SIUnit<Meter>>) : this(
        xRange.start,
        yRange.start,
        xRange.endInclusive - xRange.start,
        yRange.endInclusive - yRange.start
    )

    constructor(one: Translation2d, two: Translation2d) : this(
        one.x.meter.safeRangeTo(two.x.meter),
        one.y.meter.safeRangeTo(two.y.meter)
    )

    fun isIn(r: Rectangle2d) =
        x < r.x + r.w && x + w > r.x && y < r.y + r.h && y + h > r.y

    fun isWithin(r: Rectangle2d) = r.x in x..(x + w - r.w) && r.y in y..(y + h - r.h)

    operator fun contains(p: Translation2d) = p.x.meter in x..(x + w) && p.y.meter in y..(y + h)

    fun doesCollide(rectangle: Rectangle2d, translation: Translation2d): Boolean {
        if (translation.x == 0.0 && translation.y== 0.0) return false
        // Check if its even in range
        val boxRect = Rectangle2d(
            if (translation.x > 0) rectangle.x else rectangle.x + translation.x.meter,
            if (translation.y > 0) rectangle.x else rectangle.x + translation.y.meter,
            if (translation.x > 0) translation.x.meter + rectangle.w else rectangle.w - translation.x.meter,
            if (translation.y > 0) translation.y.meter + rectangle.h else rectangle.h - translation.y.meter
        )
        //println(boxRect)
        if (!boxRect.isIn(this)) return false
        // AABB collision
        // Calculate distances
        val xInvEntry: Double
        val xInvExit: Double
        val yInvEntry: Double
        val yInvExit: Double
        if (translation.x > 0.0) {
            xInvEntry = (x - (rectangle.x + rectangle.w)).value
            xInvExit = ((x + w) - rectangle.x).value
        } else {
            xInvEntry = ((x + w) - rectangle.x).value
            xInvExit = (x - (rectangle.x + rectangle.w)).value
        }
        if (translation.y > 0.0) {
            yInvEntry = (y - (rectangle.y + rectangle.h)).value
            yInvExit = ((y + h) - rectangle.y).value
        } else {
            yInvEntry = ((y + h) - rectangle.y).value
            yInvExit = (y - (rectangle.y + rectangle.h)).value
        }
        // Find time of collisions
        val xEntry: Double
        val xExit: Double
        val yEntry: Double
        val yExit: Double
        if (translation.x == 0.0) {
            xEntry = Double.NEGATIVE_INFINITY
            xExit = Double.POSITIVE_INFINITY
        } else {
            xEntry = xInvEntry / translation.x
            xExit = xInvExit / translation.x
        }
        if (translation.y == 0.0) {
            yEntry = Double.NEGATIVE_INFINITY
            yExit = Double.POSITIVE_INFINITY
        } else {
            yEntry = yInvEntry / translation.y
            yExit = yInvExit / translation.y
        }
        val entryTime = max(xEntry, yEntry)
        val exitTime = min(xExit, yExit)

        return entryTime <= exitTime && (xEntry >= 0.0 || yEntry >= 0.0) && (xEntry < 1.0 || yEntry < 1.0)
    }
}