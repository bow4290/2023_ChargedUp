package frc.robot.subsystems

import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase

private const val LED_LENGTH = 50
private const val MAX_CURRENT = 1500.0 // See VRM datasheet
private const val ACHIEVABLE_CURRENT = (LED_LENGTH * 60).toDouble()
private const val MAX_TOTAL_BRIGHTNESS = MAX_CURRENT / ACHIEVABLE_CURRENT * LED_LENGTH * "RGB".length

class LED : SubsystemBase() {
    private val led: AddressableLED = AddressableLED(0)
    private val buffer: AddressableLEDBuffer = AddressableLEDBuffer(LED_LENGTH)

    init {
        led.setLength(LED_LENGTH)
        solid(Color.kGreen)
        show()
        led.start()
    }

    private fun setPixel(i: Int, c: Color?) {
        buffer.setLED(i, c)
    }

    private fun solid(c: Color?) {
        for (i in 0 until LED_LENGTH) {
            setPixel(i, c)
        }
    }


    /** Modified colors in buffer to prevent too much current being used  */
    private fun preprocessBuffer() {
        var sum = 0.0
        for (i in 0 until LED_LENGTH) {
            val color = buffer.getLED(i)
            sum += color.red + color.green + color.blue
        }
        val div = sum / MAX_TOTAL_BRIGHTNESS
        if (div > 1) {
            // If we will consume too much current, unbrighten colors
            for (i in 0 until LED_LENGTH) {
                val color = buffer.getLED(i)
                buffer.setLED(i, Color(color.red / div, color.green / div, color.blue / div))
            }
            println("Warning: LED strip exceeded limit by $div")
        }
    }

    /**
     * Calls preprocessBuffer and then sets LED data. The buffer is not guaranteed to remain the same
     * after calling this.
     */
    private fun show() {
        preprocessBuffer()
        led.setData(buffer)
    }

    fun setLEDsCommand(c: Color?): Command {
        return runOnce {
            solid(c)
            show()
        }
    }
}