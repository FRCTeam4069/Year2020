package frc.team4069.robot.util

import frc.team4069.saturn.lib.types.Interpolatable
import java.util.*

/**
 * A class backed by a tree map that can interpolate entries if a specific value was not specified.
 *
 * @param V The value stored in this map. Value must be [Interpolatable]
 */
class InterpolatableTreeMap<V: Interpolatable<V>>(private val map: TreeMap<Double, V>) : NavigableMap<Double, V> by map {
    /**
     * Get the value associated with the given [key], attempting to interpolate if a value for [key] isn't found
     *
     * @return The value (interpolated or otherwise) if it can be found, null otherwise (e.g. if [key] is outside the min..max bound of all the contained entries.)
     */
    override fun get(key: Double): V? {
        // Try to get a value with the key
        val value = map[key]
        // If it exists, we're good
        return if (value == null) {
            // Get the upper and lower bounds. If none exist simply return null
            val (floorKey, floorValue) = floorEntry(key) ?: return null
            val (ceilKey, ceilValue) = ceilingEntry(key) ?: return null

            // Interpolate between the bounds
            return floorValue.interpolate(ceilValue, (key - floorKey) / (ceilKey - floorKey))
        } else value
    }
}

/**
 * Extrapolate for the given [key]. Requires that the given [key] be outside the min..max range of this map.
 *
 * @param key The key to extrapolate a value for.
 * @return The extrapolated value if the key is outside the domain of this map, null otherwise. In cases where this function returns null, a simple [get] will work.
 */
fun <V> InterpolatableTreeMap<V>.extrapolate(key: Double): V?
where V: Interpolatable<V>,
      V: Extrapolatable<V>
{
    val minKey = keys.min() ?: return null
    val maxKey = keys.max() ?: return null

    return when {
        key < minKey -> {
            val minValue = this[minKey]!!
            val (min2Key, min2Value) = ceilingEntry(minKey + 1E-3)

            return minValue.extrapolate(min2Value, (key - minKey) / (min2Key - minKey))
        }
        key > maxKey -> {
            val maxValue = this[maxKey]!!
            val (max2Key, max2Value) = floorEntry(maxKey - 1E-3)

            return max2Value.extrapolate(maxValue, (key - max2Key) / (max2Key - maxKey))
        }
        else -> {
            null
        }
    }
}

/**
 * Function to construct an [InterpolatableTreeMap] out of pairs of entries. Acts like [mapOf] in the Kotlin stdlib
 *
 * @return an [InterpolatableTreeMap] filled with the given key/value pairs.
 */
fun <V: Interpolatable<V>> interpolatableMapOf(vararg values: Pair<Double, V>): InterpolatableTreeMap<V> {
    val map = TreeMap<Double, V>()

    for ((key, value) in values) {
        map[key] = value
    }

    return InterpolatableTreeMap(map)
}
