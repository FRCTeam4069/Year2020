package frc.team4069.robot

import kotlinx.serialization.Serializable

@Serializable
data class PublishedData(
    val enabled: Boolean,
    val time: Double,
    val topicNames: List<String>,
    val topics: Map<String, Double>
) {
    // Infer topic names from topics map
    constructor(enabled: Boolean, time: Double, topics: Map<String, Double>) : this(enabled, time, topics.keys.toList(), topics)
}

