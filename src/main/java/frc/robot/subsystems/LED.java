package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  private final AddressableLED led;
  private final AddressableLEDBuffer buffer;
  private static final int length = 50;

  public LED() {
    buffer = new AddressableLEDBuffer(length);
    led = new AddressableLED(0);
    led.setLength(length);

    solid(Color.kGreen);
    show();
    led.start();
  }

  public void setPixel(int i, Color c) {
    buffer.setLED(i, c);
  }

  public void solid(Color c) {
    for (int i = 0; i < length; i++) {
      setPixel(i, c);
    }
  }

  private static final double maxCurrent = 1500; // See VRM datasheet
  private static final double theoreticalAchievableCurrent = length * 60;
  private static final double maxTotalBrightness =
      (maxCurrent / theoreticalAchievableCurrent) * length * ("RGB").length();
  private static final double safetyMarginThing = 0;

  /**
   * Modified colors in buffer to prevent too much current being used
   */
  public void preprocessBuffer() {
    double sum = 0;
    for (int i = 0; i < length; i++) {
      var color = buffer.getLED(i);
      sum += color.red + color.green + color.blue;
    }
    double div = sum / maxTotalBrightness + safetyMarginThing;
    if (div > 1) {
      // If we will consume too much current, unbrighten colors
      for (int i = 0; i < length; i++) {
        var color = buffer.getLED(i);
        buffer.setLED(i, new Color(color.red / div, color.green / div, color.blue / div));
      }
      System.out.println("Warning: LED strip exceeded limit by " + div);
    }
  }

  /**
   * Calls preprocessBuffer and then sets LED data.
   * The buffer is not guaranteed to remain the same after calling this.
   */
  public void show() {
    preprocessBuffer();
    led.setData(buffer);
  }

  public Command setLEDsCommand(Color c) {
    return runOnce(
        () -> {
          solid(c);
          show();
        });
  }
}
