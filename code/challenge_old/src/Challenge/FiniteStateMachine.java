package Challenge;

import java.util.*;
import Challenge.*;

import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.*;
import org.ros.message.challenge_msgs.*;
import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

public class FiniteStateMachine implements NodeMain {
	private enum State {
		FIND_WALL,
		ALIGN_WITH_WALL,
		WALL_AHEAD,
		TURN_AT_WALL_END,
		COLLISION_STATE,
		ALIGN_WITH_BLOCK,
		BLOCK_ALIGNED,
		COLLECTION,
		COMPUTE_PATH,
		NAVIGATE_PATH,
		CONSTRUCT
	};

	// State
	private State state;
	private String subState = null;

	// State fields
	private long stateStartTime = 0;
	private double locX = 0;
	private double locY = 0;
	private double locTheta = 0;
	private List<Double> frontSonarBuffer;
	private List<Double> backSonarBuffer;
	private double frontSonar = -1;
	private double backSonar = -1;
	private boolean bumpLeft = false;
	private boolean bumpRight = false;

	// ROS Subscribers
	public Subscriber<org.ros.message.rss_msgs.SonarMsg> frontSonarSub;
	public Subscriber<org.ros.message.rss_msgs.SonarMsg> backSonarSub;
	public Subscriber<org.ros.message.rss_msgs.BumpMsg> bumpSub;
	public Subscriber<org.ros.message.rss_msgs.OdometryMsg> odoSub;

	// ROS Publishers
	public Publisher<org.ros.message.std_msgs.String> statePub;

	public FiniteStateMachine() {
		changeState(State.FIND_WALL);
	}

	@Override
	public void onStart(Node node) {
		bumpSub = node.newSubscriber("/rss/BumpSensors", "rss_msgs/BumpMsg");
		frontSonarSub = node.newSubscriber("/rss/Sonars/Front", "rss_msgs/SonarMsg");
		backSonarSub = node.newSubscriber("/rss/Sonars/Back", "rss_msgs/SonarMsg");
		odoSub = node.newSubscriber("/rss/odometry", "rss_msgs/OdometryMsg");

		statePub = node.newPublisher("/rss/state", "std_msgs/String");

		frontSonarSub.addMessageListener(new MessageListener<org.ros.message.rss_msgs.SonarMsg>() {
			@Override
			public void onNewMessage(org.ros.message.rss_msgs.SonarMsg message) {
				
			}
		});

		backSonarSub.addMessageListener(new MessageListener<org.ros.message.rss_msgs.SonarMsg>() {
			@Override
			public void onNewMessage(org.ros.message.rss_msgs.SonarMsg message) {

			}
		});

		bumpSub.addMessageListener(new MessageListener<org.ros.message.rss_msgs.BumpMsg>() {
			@Override
			public void onNewMessage(org.ros.message.rss_msgs.BumpMsg message) {
				bumpLeft = message.left;
				bumpRight = message.right;
			}
		});

		odoSub.addMessageListener(new MessageListener<org.ros.message.rss_msgs.OdometryMsg>() {
			@Override
			public void onNewMessage(org.ros.message.rss_msgs.OdometryMsg message) {
				locX = message.x;
				locY = message.y;
				locTheta = message.theta;
			}
		});

		Thread stateThread = new Thread(new Runnable(){
			@Override
			public void run(){
				updateState();
				//publishState();
				try {
					Thread.sleep(10);
				} catch (Exception exc){
					exc.printStackTrace();
				}
			}
		});

		stateThread.start();
	}

	private void updateSonars(double val, boolean isFront){
		if (isFront){
			frontSonarBuffer.add(val);
			if (frontSonarBuffer.size() > 5){
				frontSonarBuffer.remove(0);
			}

			double tempMean = 0;
			for (Double reading : frontSonarBuffer){
				tempMean += reading;
			}

			tempMean /= frontSonarBuffer.size();

			frontSonar = tempMean;
		} else {
			backSonarBuffer.add(val);
			if (backSonarBuffer.size() > 5){
				backSonarBuffer.remove(0);
			}

			double tempMean = 0;
			for (Double reading : backSonarBuffer){
				tempMean += reading;
			}

			tempMean /= backSonarBuffer.size();

			backSonar = tempMean;
		}
	}

	private void updateState(){
		switch (state) {
			case FIND_WALL:
				break;
			case ALIGN_WITH_WALL:
				break;
			case WALL_AHEAD:
				break;
			case TURN_AT_WALL_END:
				break;
			case COLLISION_STATE:
				break;
			case ALIGN_WITH_BLOCK:
				break;
			case BLOCK_ALIGNED:
				break;
			case COLLECTION:
				break;
			case COMPUTE_PATH:
				break;
			case NAVIGATE_PATH:
				break;
			case CONSTRUCT:
				break;
			default:
				break;
		}
	}
/*		if (state == STOP_ON_BUMP){
			if (bumpLeft || bumpRight){
				setVelocities(0, 0);
			} else {
				setVelocities(0.3, 0);
			}
		} else if (state == ALIGN_ON_BUMP){
			System.out.println("ALIGN_ON_BUMP");
			if (bumpLeft || bumpRight){
				state = ALIGNING;
				publishState();
			} else {
				setVelocities(0.2, 0);
			}
		} else if (state == ALIGNING){
			System.out.println("ALIGNING");
			if (bumpLeft && bumpRight){
				state = ALIGNED;
				publishState();
				setVelocities(0, 0);
				//goalPoint = new Point(locX - desiredWallDist * Math.cos(locTheta), locY - desiredWallDist * Math.sin(locTheta));
				goalPoint = new Point(locX, locY);
			} else if (bumpLeft){
				setVelocities(0, -0.02);
			} else if (bumpRight){
				setVelocities(0, 0.02);
			} else {
				setVelocities(0.02, 0);
			}
		} else if (state == ALIGNED){
			System.out.println("ALIGNED");
			System.out.println("Goal Point: " + goalPoint.x + ", " + goalPoint.y);
			System.out.println("Current Point:" + locX + ", " + locY);

			if ((locX - goalPoint.x) * (locX - goalPoint.x) +
				(locY - goalPoint.y) * (locY - goalPoint.y) < desiredWallDist * desiredWallDist){
				setVelocities(-0.5*desiredWallDist, 0);
			} else {
				state = ALIGNED_AND_ROTATING;
				anglePID = new PIDController(0.3, 0, 0);
				double angle = locTheta - Math.PI/2;
				if (angle < 0){
					angle += 2 * Math.PI;
				}
				goalPoint = new Point(locX, locY, angle);
				setVelocities(0, 0);
			}
			publishState();

		} else if (state == ALIGNED_AND_ROTATING){
			System.out.println("ALIGNED_AND_ROTATING");
			System.out.println("CURR " + locTheta);
			System.out.println("GOAL " + goalPoint.theta);
			double error = goalPoint.theta - locTheta;

			if (Math.abs(error) > 0.05){
				setVelocities(0, anglePID.update(error));
			} else {
				setVelocities(0, 0);
				state = ALIGNED_AND_ROTATED;
				publishState();
			}
			//System.out.println("CURR " + locTheta);
			//System.out.println("GOAL " + goalPoint.theta);
			//if (Math.abs(locTheta - goalPoint.theta) > 0.25){
			//	setVelocities(0, -0.15);
			//} else {
			//	setVelocities(0, 0);
			//}
		} else if (state == ALIGNED_AND_ROTATED){
			System.out.println("ALIGNED_AND_ROTATED");
			state = BACKING_UP;
			publishState();
		} else if (state == BACKING_UP){
			//If we see an obstacle and are in BACKING_UP State, start backing up
			System.out.println("BACKING_UP");
			//double currBack = backSonar;
			//double currFront = frontSonar;
			//updateWallReadings();
			if(obstacle && Math.max(backSonar, frontSonar) < 0.6){
				setVelocities(-.3, -trackingPID.update(capped(backSonar) - capped(frontSonar)));
				//setVelocities(-.3, -trackingPID.update(capped(currBack) - capped(currFront)));
				//setVelocities(-0.3, 0);
			} else {
				setVelocities(0,0);
				state = FINDING_WALL;
				publishState();
			}
		} else if (state == FINDING_WALL){
			System.out.println("FINDING_WALL");
			updateWallReadings();
			if(!obstacle){
				setVelocities(.3, 0.0);
			} else {
				//setVelocities(0.0,0.0);
				state = TRACKING_WALL;
				publishState();
			}
		} else if (state == TRACKING_WALL){
			System.out.println("TRACKING_WALL");
			System.out.println(obstacle);
			//double currBack = backSonar;
			//double currFront = frontSonar;
			//updateWallReadings();
			//if(!(Math.min(frontSonar, backSonar) > MAX_OBSTACLE || Math.max(frontSonar, backSonar) < MIN_OBSTACLE)){
			if (obstacle && Math.max(backSonar, frontSonar) < 0.6){
				//setVelocities(.3, -trackingPID.update(0.37 - 0.5 * (capped(backSonar) + capped(frontSonar)));
				setVelocities(.3, -trackingPID.update(capped(backSonar) - capped(frontSonar)));
				//setVelocities(0.3, -trackingPID.update(capped(currBack) - capped(currFront)));
				double translationError = desiredWallDist - 0.5*(frontSonar + backSonar);
				double rotationError = (backSonar - frontSonar);
				long timeElapsedMillis = System.currentTimeMillis(); //TODO improve
				try {
					outFile.write("" + timeElapsedMillis + ", " + translationError + ", " + rotationError + "\n");
				} catch (IOException e) {
					System.out.println("Error reading file: " + e);
				}
			} else {
				try {
					outFile.close();
					System.out.println("Closing outFile");
				} catch (IOException e) {
					System.out.println("Error closing file: " + e);
				}
				System.out.println("drawing segment");
				drawObstacleSegment(totalObstaclePoints.get(totalObstaclePoints.size() - 1), totalLines.get(totalLines.size() - 1));
				setVelocities(0,0);
				state = WALL_ENDED;
				publishState();
			}
		} else if (state == WALL_ENDED){
			System.out.println("WALL_ENDED");
			try {
				outFile.close();
			} catch (IOException e) {
				e.printStackTrace();
			}
			if (prevWallAngle < -8){
				prevWallAngle = locTheta;
				travelledAngle = 0;
			} else {
				double angle = locTheta - prevWallAngle;
				if (angle < 0){
					angle += 2 * Math.PI;
				} else if (angle > 2 * Math.PI){
					angle -= 2 * Math.PI;
				}
				travelledAngle += angle;
				prevWallAngle = locTheta;
			}

			if (travelledAngle > 2 * Math.PI){
				state = DONE;
				publishState();
			} else {
				state = CIRCLE_TO_BUMP;
				publishState();
			}

		} else if (state == CIRCLE_TO_BUMP){
			System.out.println("CIRCLE_TO_BUMP");
			if (bumpLeft || bumpRight){
				state = ALIGNING;
				publishState();
			} else {
				setVelocities(0.4, 0.2);
			}
		} else if (state == DONE){
			try {
				outFile.close();
				System.out.println("Closing outFile");
			} catch (IOException e) {
				System.out.println("Error closing file: " + e);
			}
			System.out.println("DONE");
			setVelocities(0, 0);
		}
	}
	*/

	private long getStateTime(){
		return System.currentTimeMillis() - stateStartTime;
	}

	private void changeState(State newState){
		stateStartTime = System.currentTimeMillis();
		state = newState;
	}

	/*private void publishState(){
		org.ros.message.std_msgs.String msg = new org.ros.message.std_msgs.String();
		msg.data = state;
		statePub.publish(msg);
	}*/

	@Override
	public void onShutdown(Node node) {
		if (node != null) {
			node.shutdown();
		}
	}

	@Override
	public void onShutdownComplete(Node node) {

	}

	@Override
	public GraphName getDefaultNodeName() {
		return new GraphName("rss/finitestatemachine");
	}
}
