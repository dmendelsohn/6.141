package MotorControl;

/**
 * <p>Open loop wheel velocity controller.
 * Code for Motion Control Lab Part 9 </p>
 *
 * @author vona
 * @author prentice
 **/
public class WheelVelocityControllerPWM extends WheelVelocityController {

  /**
   * {@inheritDoc}
   *
   * <p>This impl implements simple open-loop control.</p>
   **/
  public double controlStep() {
	// directly set PWM
    return desiredAngularVelocity;
  }

  /**
   * {@inheritDoc}
   *
   * <p>This impl returns "PWM".</p>
   **/
  public String getName() {
    return "PWM";
  }
}
