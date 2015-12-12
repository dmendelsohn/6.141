package Chassis;

//import MotorControlSolution.*;
import MotorControl.*;

/**
 * <p>A whole-robot position controller.</p>
 **/
public class RobotPositionController {

	/**
	 * <p> Constant indicating the number of ticks per revolution.</p>
	 **/
	protected final int TICKS_PER_REVOLUTION = 131000;

	/**
	 * <p> Constant indicating the wheel radius in meters.</p>
	 **/
	protected final double WHEEL_RADIUS_IN_M = 0.123825 / 2.0;

	/**
	 * <p>The whole-robot velocity controller.</p>
	 **/
	protected RobotVelocityController robotVelocityController;

	/**
	 * <p>Total ticks since reset, positive means corresp side of robot moved
	 * forward.</p>
	 **/
	protected double[] totalTicks = new double[2];

	/**
	 * <p>Total elapsed time since reset in seconds.</p>
	 **/
	protected double totalTime = 0.0;

	/**
	 * <p>Time in seconds since last update.</p>
	 **/
	protected double sampleTime;

	/**
	 * <p>An abstract gain; meaning depends on the particular subclass
	 * implementation.</p>
	 **/
	protected double gain = 1.0;

	/**
	 * <p>The robot.</p>
	 **/
	protected OdometryRobot robot;

	/**
	 * <p>Create a new position controller for a robot.</p>
	 *
	 * @param robot the robot, not null
	 **/
	public RobotPositionController(OdometryRobot robot) {
		this.robot = robot;
	}

	/**
	 * <p>Translate at the specified speed for the specified distance.</p>
	 *
	 * <p>Blocks until the motion is complete or errored.</p>
	 *
	 * @param speed the desired robot motion speed in m/s
	 * @param distance the desired distance to move in meters, relative to the
	 * robot's pose at time of call.
	 *
	 * @return true iff motion was successful
	 **/
	public boolean translate(double speed, double distance) {
		boolean ok = true;

		double angularVel = speed / (WHEEL_RADIUS_IN_M*2*Math.PI);
		double totalTicksAVG, distanceTraveled;

		// compute initial distance travelled at the beginning of the task
		double initialTicks = (totalTicks[RobotBase.LEFT] + totalTicks[RobotBase.RIGHT])/2;
		double initialDistance = (initialTicks / TICKS_PER_REVOLUTION) * WHEEL_RADIUS_IN_M * 2 * Math.PI;

		System.out.println(angularVel);

		// set desired motor speeds equal for straight movement
		synchronized (robotVelocityController){
			robotVelocityController.setDesiredAngularVelocity(angularVel, angularVel);
		}

		while(true){
			// determine total distance so far travelled
			totalTicksAVG = (totalTicks[RobotBase.LEFT] + totalTicks[RobotBase.RIGHT])/2;
			distanceTraveled = (totalTicksAVG / TICKS_PER_REVOLUTION) * WHEEL_RADIUS_IN_M * 2 * Math.PI;

			// if travelled far enough beyond initial distance then stop
			if (distanceTraveled >= distance + initialDistance){
				synchronized (robotVelocityController){
					// set desired wheel speeds equal to zero
					robotVelocityController.setDesiredAngularVelocity(0.0, 0.0);

					// reset control variables since starting new task
					robotVelocityController.resetController();
				}
				try {
					Thread.sleep(1000);
				} catch (Exception exc){
					System.out.println("Exception found");
				}
				return ok;
			}
		}
	}

	/**
	 * <p>Rotate at the specified speed for the specified angle.</p>
	 *
	 * <p>Blocks until the motion is complete or errored.</p>
	 *
	 * @param speed the desired robot motion speed in radians/s
	 * @param angle the desired angle to rotate in radians, relative to the
	 * robot's pose at time of call.
	 *
	 * @return true iff motion was successful
	 **/
	public boolean rotate(double speed, double angle){
		boolean ok = true;
		double angularVel = speed / (WHEEL_RADIUS_IN_M * 2 * Math.PI);

		// determine initial difference in distance travelled by wheels (initial angle measure)
		double initialTickDifference = totalTicks[RobotBase.LEFT] - totalTicks[RobotBase.RIGHT];
		double tickDifference, angularDistance, angleMoved;

		// set desired motor speeds for turning
		synchronized (robotVelocityController){
			robotVelocityController.setDesiredAngularVelocity(angularVel, -angularVel);
		}

		while(true) {
			// determine total distance displaced (angle measure) from initial value and current
			tickDifference = totalTicks[RobotBase.LEFT] - totalTicks[RobotBase.RIGHT] - initialTickDifference;
			angularDistance = (tickDifference / TICKS_PER_REVOLUTION) * WHEEL_RADIUS_IN_M * 2 * Math.PI;
			angleMoved = angularDistance / 0.415;

			// if have turned enough then stop
			if (angleMoved > angle){
				synchronized (robotVelocityController){
					// set desired wheel speeds equal to zero
					robotVelocityController.setDesiredAngularVelocity(0.0, 0.0);

					// reset control variables since starting new task
					robotVelocityController.resetController();
				}
				try {
					Thread.sleep(1000);
				} catch (Exception exc){
					System.out.println("Exception found");
				}
				return ok;
			}
		}
	}

	/**
	 * <p>If position control is closed-loop, this computes the new left and
	 * right velocity commands and issues them to {@link
	 * #robotVelocityController}.</p>
	 **/
	public synchronized void controlStep() {

		if (robotVelocityController == null)
			return;

		if (!robot.motorsEnabled() || robot.estopped())
			return;
	}

	/**
	 * <p>Set the whole-robot velocity controller.</p>
	 *
	 * <p>This is called automatically by {@link OdometeryRobot}.</p>
	 *
	 * @param vc the whole-robot velocity controller
	 **/
	public void setRobotVelocityController(RobotVelocityController vc) {
		robotVelocityController = vc;
	}

	/**
	 * <p>Set {@link #gain}.</p>
	 *
	 * @param g the new gain
	 **/
	public void setGain(double g) {
		gain = g;
	}

	/**
	 * <p>Get {@link #gain}.</p>
	 *
	 * @return gain
	 **/
	public double getGain() {
		return gain;
	}

	/**
	 * <p>Update feedback and sample time.</p>
	 *
	 * @param time the time in seconds since the last update, saved to {@link
	 * #sampleTime}
	 * @param leftTicks left encoder ticks since last update, positive means
	 * corresp side of robot rolled forward
	 * @param rightTicks right encoder ticks since last update, positive means
	 * corresp side of robot robot rolled forward
	 **/
	public synchronized void update(double time,
			double leftTicks, double rightTicks) {

		sampleTime = time;

		totalTicks[RobotBase.LEFT] += leftTicks;
		totalTicks[RobotBase.RIGHT] += rightTicks;
		totalTime += time;
	}

	/**
	 *
	 * <p>This indicates that the robot is starting a new task and needs
	 * to reset control variables.</p>
	 **/
	public void resetController(){
		// output message for debugging
		System.out.println("Not implemented");
	}
}
