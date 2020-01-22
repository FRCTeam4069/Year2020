package frc.team4069.robot.subsystems

import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj.Spark
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.util.Color
import frc.team4069.saturn.lib.mathematics.units.SIUnit
import frc.team4069.saturn.lib.mathematics.units.Second
import frc.team4069.saturn.lib.mathematics.units.conversions.second
import frc.team4069.saturn.lib.mathematics.units.derived.Hertz
import frc.team4069.saturn.lib.mathematics.units.second
import frc.team4069.saturn.lib.util.DeltaTime
import frc.team4069.saturn.lib.util.loopFrequency
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.Job
import kotlinx.coroutines.ObsoleteCoroutinesApi
import kotlinx.coroutines.channels.actor
import kotlinx.coroutines.channels.sendBlocking
import kotlinx.coroutines.launch
import java.lang.IllegalArgumentException
import kotlin.math.PI
import kotlin.math.sin

@UseExperimental(ObsoleteCoroutinesApi::class)
object LED {

    const val LED_PWM_PORT = 1
    const val LED_STRIP_LENGTH = 10

    private val leds = AddressableLED(LED_PWM_PORT)

    init {
        leds.setLength(LED_STRIP_LENGTH)
    }

    /**
     * Coroutine that controls demands related to the brightness of the LEDs
     *
     * Messages can be posted through [updateBrightness] to change the behaviour
     */
    private val brightnessActor = GlobalScope.actor<BrightnessDemand> {
        var childJob: Job? = null
        val ledPwr = Spark(0)

        for(msg in channel) {
            childJob?.cancel()
            childJob = null
            when(msg) {
                BrightnessDemand.On -> {
                    ledPwr.set(1.0)
                }
                BrightnessDemand.Off -> {
                    ledPwr.set(0.0)
                }
                is BrightnessDemand.Sinusoidal -> {
                    ledPwr.set(0.0)
                    childJob = launch {
                        var t = 0.0
                        val sinFreq = msg.frequency.value
                        val deltaTime = DeltaTime()
                        loopFrequency(100) {
                            val dt = deltaTime.updateTime(Timer.getFPGATimestamp().second).second
                            t += dt
                            val output = (sin(PI * sinFreq * t) + 1) / 2.0
                            ledPwr.set(output)
                        }
                    }
                }
                is BrightnessDemand.Square -> {
                    ledPwr.set(0.0)
                    childJob = launch {
                        var elapsed = 0.0
                        val desired = msg.peakTime.value
                        val deltaTime = DeltaTime()
                        var on = false
                        loopFrequency(100) {
                            val dt = deltaTime.updateTime(Timer.getFPGATimestamp().second).second
                            elapsed += dt
                            if(elapsed >= desired) {
                                on = !on
                                elapsed = 0.0
                                ledPwr.set(if(on) 1.0 else 0.0)
                            }
                        }
                    }
                }
            }
        }
    }

    fun updateBrightness(demand: BrightnessDemand) {
        brightnessActor.sendBlocking(demand)
    }

    fun updateColours(colours: List<Color>) {
        if(colours.size < LED_STRIP_LENGTH) {
            throw IllegalArgumentException("Colours list must be $LED_STRIP_LENGTH entries long.")
        }

        val buf = AddressableLEDBuffer(LED_STRIP_LENGTH)

        for(i in 0 until LED_STRIP_LENGTH) {
            buf.setLED(i, colours[i])
        }

        leds.setData(buf)
    }

    sealed class BrightnessDemand {
        object On : BrightnessDemand()
        object Off : BrightnessDemand()

        // Based on output = |sin(pi*f*t)|
        data class Sinusoidal(val frequency: SIUnit<Hertz>) : BrightnessDemand() {
            init {
                if(frequency.value <= 0.0) {
                    throw IllegalArgumentException("Frequency must be strictly positive")
                }
            }
        }

        // Cycles 0-1, with flat parts lastting [peakTime]
        data class Square(val peakTime: SIUnit<Second>) : BrightnessDemand()
    }
}

