package frc.team4069.robot.subsystems

import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
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


    /**
     * Coroutine that controls demands related to the brightness of the LEDs
     *
     * Messages can be posted through [updateBrightness] to change the behaviour
     */
    private val ledControlActor =  GlobalScope.actor<LedDemand> {
        var childJob: Job? = null

        val leds = AddressableLED(LED_PWM_PORT)
        leds.setLength(LED_STRIP_LENGTH)
        leds.start()

        var demandBuffer: AddressableLEDBuffer

        for(msg in channel) {
            childJob?.cancel()
            childJob = null
            when(msg) {
                is LedDemand.On -> {
                    demandBuffer = msg.colours.toBuffer()
                    leds.setData(demandBuffer)
                }
                LedDemand.Off -> {
                    val l = mutableListOf<Color>()
                    repeat(LED_STRIP_LENGTH) {
                        l += Color(0.0, 0.0, 0.0)
                    }
                    demandBuffer = l.toBuffer()
                    leds.setData(demandBuffer)
                }
                is LedDemand.Sinusoidal -> {
                    val init = msg.colours.map { Color(0.0, 0.0, 0.0) }
                    demandBuffer = init.toBuffer()
                    leds.setData(demandBuffer)

                    childJob = launch {
                        val base = msg.colours
                        val freq = msg.frequency.value
                        val dt = 0.02
                        var t = 0.0

                        loopFrequency(50) {
                            t += dt
                            val scale = (sin(PI*freq*t) + 1.0) / 2.0
                            demandBuffer = base.map { Color(it.red * scale, it.green * scale, it.blue * scale) }.toBuffer()
                            leds.setData(demandBuffer)
                        }
                    }
                }
                is LedDemand.Square -> {
                    childJob = launch {
                        val nullBuf = msg.colours.map { Color(0.0, 0.0, 0.0) }.toBuffer()
                        val colours = msg.colours.toBuffer()
                        val dt = 0.02
                        var elapsed = 0.0
                        val peakTime = msg.peakTime.value
                        var on = false
                        leds.setData(nullBuf)

                        loopFrequency(50) {
                            elapsed += dt
                            if(elapsed >= peakTime) {
                                if(on) {
                                    on = false
                                    leds.stop()
                                } else {
                                    on = true
                                    leds.start()
                                    leds.setData(colours)
                                }

                                elapsed = 0.0
                            }
                        }
                    }
                }
            }
        }
    }

    fun updateDemand(demand: LedDemand) {
        ledControlActor.sendBlocking(demand)
    }

    fun List<Color>.toBuffer(): AddressableLEDBuffer {
        if(this.size < LED_STRIP_LENGTH) {
            throw IllegalArgumentException("List must be at least $LED_STRIP_LENGTH elements long.")
        }

        val buffer = AddressableLEDBuffer(LED_STRIP_LENGTH)

        for(i in 0 until LED_STRIP_LENGTH) {
            buffer.setLED(i, this[i])
        }

        return buffer
    }

    sealed class LedDemand {
        data class On(val colours: List<Color>) : LedDemand()
        object Off : LedDemand()

        // Based on output = |sin(pi*f*t)|
        data class Sinusoidal(val frequency: SIUnit<Hertz>, val colours: List<Color>) : LedDemand() {
            init {
                if(frequency.value <= 0.0) {
                    throw IllegalArgumentException("Frequency must be strictly positive")
                }
            }
        }

        // Cycles 0-1, with flat parts lastting [peakTime]
        data class Square(val peakTime: SIUnit<Second>, val colours: List<Color>) : LedDemand()
    }
}

