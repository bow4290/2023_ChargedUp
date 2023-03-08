package frc.lib.math;

import edu.wpi.first.wpilibj.Timer;

public class RateMeasurer {
  double previousMeasurement;
  Timer measurementTimer = new Timer();
  double rate;

  public void init(double measurement) {
    previousMeasurement = measurement;
    measurementTimer.start();
  }

  public void addMeasurement(double measurement) {
    double lastPreviousMeasurement = previousMeasurement;
    previousMeasurement = measurement;
    double time = measurementTimer.get();
    measurementTimer.reset();
    rate = (measurement - lastPreviousMeasurement) / time;
  }

  public double getRate() {
    return rate;
  }

  public double getRate(double measurement) {
    addMeasurement(measurement);
    return getRate();
  }
}
