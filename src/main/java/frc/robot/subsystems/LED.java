package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import java.util.Map;

public class LED extends SubsystemBase {
  private static final int port = 1;
  private static final int length = 50;
  private final AddressableLED led = new AddressableLED(port);
  private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(length);
  private RobotContainer bot; // This probably is not great practice but it works and is convenient

  public LED() {
    led.setLength(length);
    led.start();
  }

  public void setBot(RobotContainer bot) {
    this.bot = bot;
    startRunningLEDs();
  }

  public void setPixel(int i, Color c) {
    buffer.setLED(i, c);
  }

  public void setSolid(Color c) {
    for (int i = 0; i < length; i++) {
      setPixel(i, c);
    }
  }

  private static final double maxCurrent = 1550;
  private static final double theoreticalAchievableCurrent = length * 60;
  private static final double maxTotalBrightness =
      (maxCurrent / theoreticalAchievableCurrent) * length * ("RGB").length();

  /** Modify colors in buffer to prevent too much current being used */
  public void preprocessBuffer() {
    double sum = 0;
    for (int i = 0; i < length; i++) {
      var color = buffer.getLED(i);
      sum += color.red + color.green + color.blue;
    }
    double div = sum / maxTotalBrightness;
    if (div > 1) {
      // If we will consume too much current, unbrighten colors
      for (int i = 0; i < length; i++) {
        var color = buffer.getLED(i);
        buffer.setLED(i, new Color(color.red / div, color.green / div, color.blue / div));
      }
    }
  }

  /**
   * Calls preprocessBuffer and then sets LED data. The buffer is not guaranteed to remain the same
   * after calling this.
   */
  public void show() {
    preprocessBuffer();
    led.setData(buffer);
  }

  public static final double rainbowPeriod = 1;
  public static final double rainbowIncrement = 10;

  /*
  A quick note on LED commands:
  Each individual LED pattern (rainbow, solidLEDs, etc.) is implemented as a runOnce
  **There is no state** so all the commands should just use getFPGATimestamp to determine their lighting up things
  Then the "big SelectCommand" in RobotContainer chooses the right command based on the state
   */
  private Command runLEDs(Runnable action) {
    return runOnce(
            () -> {
              action.run();
              show();
            })
        .ignoringDisable(true);
  }

  public Command rainbow() {
    return runLEDs(
        () -> {
          double time = (Timer.getFPGATimestamp() % rainbowPeriod) * (180 / rainbowPeriod);
          for (int i = 0; i < length; i++) {
            var col = Color.fromHSV((int) (time + rainbowIncrement * i), 255, 255);
            setPixel(i, col);
          }
        });
  }

  public Command snow() {
    return runLEDs(
        () -> {
          double subsection = Math.floor((Timer.getFPGATimestamp() * 2) % 10);
          for (int i = 0; i < length; i++) {
            setPixel(i, i % 10 == subsection ? Color.kBlack : Color.kWhite);
          }
        });
  }

  public Command solid(Color c) {
    return runLEDs(() -> setSolid(c));
  }

  public Trigger squareRecently;
  public Trigger crossRecently;

  private enum LEDState {
    rainbow,
    green,
    yellow,
    purple,
    red,
    snow,
    blue
  }

  private LEDState chooseLEDState() {
    // blue endgame warning thing
    if (bot.bot.isTeleopEnabled()
        && DriverStation.isFMSAttached()
        && Timer.getMatchTime() < 30
        && Timer.getMatchTime() > 28) {
      return LEDState.blue;
    }

    if (bot.s_Intake.intakeHasThing.getAsBoolean()) {
      return LEDState.green;
    }

    if (bot.bot.isAutonomousEnabled()) {
      return LEDState.snow;
    }
    if (bot.driver.square_x.getAsBoolean()) {
      return LEDState.purple;
    }
    if (bot.driver.cross_a.getAsBoolean()) {
      return LEDState.yellow;
    }

    if (squareRecently.getAsBoolean()) {
      return LEDState.purple;
    }
    if (crossRecently.getAsBoolean()) {
      return LEDState.yellow;
    } // bot.bot lol??

    // if the elbow is currently doing something non-default with the elbow, red
    if (bot.bot.isTeleopEnabled()
        && bot.s_Elbow.getCurrentCommand() != bot.s_Elbow.getDefaultCommand()) {
      return LEDState.red;
    }

    return LEDState.rainbow;
  }

  private void startRunningLEDs() {
    Commands.select(
            Map.ofEntries(
                Map.entry(LEDState.rainbow, rainbow()),
                Map.entry(LEDState.snow, snow()),
                Map.entry(LEDState.green, solid(Color.kGreen)),
                Map.entry(LEDState.yellow, solid(Color.kYellow)),
                Map.entry(LEDState.purple, solid(Color.kPurple)),
                Map.entry(LEDState.blue, solid(Color.kBlue)),
                Map.entry(LEDState.red, solid(Color.kRed))),
            this::chooseLEDState)
        .repeatedly()
        .schedule();
  }
}
