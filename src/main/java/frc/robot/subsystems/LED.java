package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  private final AddressableLED led;
  private final AddressableLEDBuffer buffer;
  private final static int length = 40;

  public LED() {
    buffer = new AddressableLEDBuffer(length);
    led = new AddressableLED(0);

    led.setData(buffer);
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

  public void show() {
    led.setData(buffer);
  }

  public Command setLEDsCommand(Color c) {
    return runOnce(() -> {solid(c); show(); });
  }
}
