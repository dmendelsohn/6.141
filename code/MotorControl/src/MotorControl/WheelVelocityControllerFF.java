package MotorControl;

/**
 * <p>Feed-forward wheel velocity controller.
 * Code for Motion Control Lab Part 9 </p>
 *
 * @author vona
 * @author prentice
 **/
public class WheelVelocityControllerFF extends WheelVelocityController {

  /**
   * {@inheritDoc}
   *
   * <p>This impl implements simple feed-forward control.</p>
   **/
	public double controlStep() {
		// directly determine required PWM to set desired speed
  		return (desiredAngularVelocity / MAX_ANGULAR_VELOCITY) * MAX_PWM;
  }

  /**
   * {@inheritDoc}
   *
   * <p>This impl returns "FF".</p>
   **/
  public String getName() {
    return "FF";
  }
}
