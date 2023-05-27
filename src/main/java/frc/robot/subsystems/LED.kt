package frc.robot.subsystems

import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import kotlin.math.floor

private const val LED_LENGTH = 50
private const val MAX_CURRENT = 1500.0 // See VRM datasheet
private const val ACHIEVABLE_CURRENT = (LED_LENGTH * 60).toDouble()
private const val MAX_TOTAL_BRIGHTNESS = MAX_CURRENT / ACHIEVABLE_CURRENT * LED_LENGTH * "RGB".length

class LED : SubsystemBase() {
    private val led: AddressableLED = AddressableLED(0)
    private val buffer: AddressableLEDBuffer = AddressableLEDBuffer(LED_LENGTH)

    init {
        led.setLength(LED_LENGTH)
        //solid(Color.kGreen)
        rainbow2().schedule()
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
            //println("Warning: LED strip exceeded limit by $div")
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

    fun rainbow(): Command {
        val period = 5;
        return run {
            var time = (Timer.getFPGATimestamp() % period) * (180 / period)
            var col = Color.fromHSV(time.toInt(), 255, 255);
            solid(col)
            show()
        }.ignoringDisable(true)
    }

    fun rainbow2(): Command {
        val period = 1;
        val increment = 10;
        return run {
            var time = (Timer.getFPGATimestamp() % period) * (180 / period)
            for (i in 0 until LED_LENGTH) {
                var col = Color.fromHSV(time.toInt() + increment * i, 255, 255);
                setPixel(i, col)
            }
            show()
        }.ignoringDisable(true)
    }

    fun sparkles(): Command {
        val period = 0.4
        return run {
            //var time = floor((Timer.getFPGATimestamp() % period) * (10 / period))
            for (i in 0 until LED_LENGTH) {
                if (Math.random() < 0.02) {
                    setPixel(i, Color.kWhite)
                } else {
                    setPixel(i, Color.kBlack)
                }
            }
            show()
        }.ignoringDisable(true)
    }

    fun snow(): Command {
        val period = 1;
        val increment = 10;
        return run {
            var time = (Timer.getFPGATimestamp() % period) * (255 / period)
            for (i in 0 until LED_LENGTH) {
                var col = Color.fromHSV(200, 255, time.toInt() + increment * i);
                setPixel(i, col)
            }
            show()
        }.ignoringDisable(true)
    }

    fun trans(): Command {
        //val period = 1;
        //val increment = 10;
        return run {
            //var time = (Timer.getFPGATimestamp() % period) * (180 / period)
            var blue = Color(91/255, 206/255, 250/255)
            var pink = Color(245/255, 169/255, 184/255)
            var white = Color(255/255, 255/255, 255/255)

            var colors = arrayListOf(blue, pink, white, pink, blue);

            for (i in 0 until LED_LENGTH) {
                setPixel(i, colors[floor(((i / 10).toDouble())).toInt()])
            }
            show()
        }.ignoringDisable(true)
    }
}