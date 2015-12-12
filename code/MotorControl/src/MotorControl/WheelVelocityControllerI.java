package MotorControl;

/**
 * <p>Closed-loop integral wheel velocity controller.
 * Code for Motion Control Lab Part 10 </p>
 *
 * @author vona
 * @author prentice
 **/
public class WheelVelocityControllerI extends WheelVelocityController {

	/**
	 * <p>The result of the previous control step.</p>
	 **/
	protected double integral = 0;
	
	/**
	 * <p> Whether or not to reset control variables at next iteration of
	 * the control loop </p>
	 **/
	protected boolean reset = false;

	/**
	 * {@inheritDoc}
	 *
	 * <p>This impl implements closed-loop integral control.</p>
	 **/
	public double controlStep() {

		double result = 0;

		// determine the error and update the integral
		// the integral is discretely approximated with resolution determined by sampleTime
		double error = desiredAngularVelocity - computeAngularVelocity();
		integral += error * sampleTime;
		
		// compute PWM output with integral
		result = gain * integral;

		// Clip output PWM with min and max
		if (result > MAX_PWM)
			result = MAX_PWM;
		if (result < -MAX_PWM)
			result = -MAX_PWM;
		
		// if control indicates starting a new task and need to reset
		// control variables, clear integral and result
		if (reset){
			integral = 0;
			result = 0;
			reset = false;
		}

		return result;
	}

	/**
	 * {@inheritDoc}
	 *
	 * <p>This indicates that the robot is starting a new task and needs
	 * to reset control variables.</p>
	 **/
	public void resetController(){
		reset = true;
	}


	/**
	 * {@inheritDoc}
	 *
	 * <p>This impl returns "I".</p>
	 **/
	public String getName() {
		return "I";
	}
}
