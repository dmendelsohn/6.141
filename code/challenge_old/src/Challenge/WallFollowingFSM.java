package Challenge;

import Drive.Drive;
import Drive.Drive.DriveCommand;

import java.util.*;

import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.*;
import org.ros.message.challenge_msgs.*;
import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

public class WallFollowingFSM implements NodeMain {
	private enum FollowingState {
		FIND_WALL,
		WALL_AHEAD,
		FOLLOW_WALL
	};

	// State
	private FollowingState state;

	// State fields
	private long stateStartTime = 0;
	private double locX = 0;
	private double locY = 0;
	private double locTheta = 0;
	private SonarBuffer sonarBuffer;
	private double wallAheadDistance = -1;
	private boolean isWallAhead = false;
	private boolean bumpLeft = false;
	private boolean bumpRight = false;

	// ROS Subscribers
	public Subscriber<org.ros.message.rss_msgs.SonarMsg> frontSonarSub;
	public Subscriber<org.ros.message.rss_msgs.SonarMsg> backSonarSub;
	public Subscriber<org.ros.message.rss_msgs.BumpMsg> bumpSub;
	public Subscriber<org.ros.message.rss_msgs.OdometryMsg> odoSub;
	public Subscriber<org.ros.message.challenge_msgs.WallAheadMsg> wallAheadSub;

	// ROS Publishers
	public Publisher<org.ros.message.std_msgs.String> statePub;
	public Publisher<org.ros.message.challenge_msgs.DriveMsg> drivePub;

	public WallFollowingFSM() {
		sonarBuffer = new SonarBuffer();
		changeState(FollowingState.FIND_WALL);
	}

	@Override
	public void onStart(Node node) {
		bumpSub = node.newSubscriber("/rss/BumpSensors", "rss_msgs/BumpMsg");
		frontSonarSub = node.newSubscriber("/rss/Sonars/Front", "rss_msgs/SonarMsg");
		backSonarSub = node.newSubscriber("/rss/Sonars/Back", "rss_msgs/SonarMsg");
		wallAheadSub = node.newSubscriber("/rss/Vision/Distance", "challenge_msgs/WallAheadMsg"); //Be careful about the topic name
		odoSub = node.newSubscriber("/rss/odometry", "rss_msgs/OdometryMsg");
		statePub = node.newPublisher("/rss/state", "std_msgs/String");
		drivePub = node.newPublisher(Drive.WALL_FOLLOWING_FSM_TOPIC, "challenge_msgs/DriveMsg");

		frontSonarSub.addMessageListener(new MessageListener<org.ros.message.rss_msgs.SonarMsg>() {
			@Override
			public void onNewMessage(org.ros.message.rss_msgs.SonarMsg message) {
				sonarBuffer.updateSonars(message.range, true);
			}
		});

		backSonarSub.addMessageListener(new MessageListener<org.ros.message.rss_msgs.SonarMsg>() {
			@Override
			public void onNewMessage(org.ros.message.rss_msgs.SonarMsg message) {
				sonarBuffer.updateSonars(message.range, false);
			}
		});

		wallAheadSub.addMessageListener(new MessageListener<org.ros.message.challenge_msgs.WallAheadMsg>() {
			@Override
			public void onNewMessage(org.ros.message.challenge_msgs.WallAheadMsg message) {
				isWallAhead = message.isWallAhead;
				wallAheadDistance = message.wallAheadDistance;
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
				publishState();
				try {
					Thread.sleep(10);
				} catch (Exception exc){
					exc.printStackTrace();
				}
			}
		});
		stateThread.start();
	}
	
	private void updateState(){
		org.ros.message.challenge_msgs.DriveMsg msg = new org.ros.message.challenge_msgs.DriveMsg();
		org.ros.message.std_msgs.String cmd = new org.ros.message.std_msgs.String();
		msg.command = cmd;
		switch (state) {
			case FIND_WALL:
				if (isWallAhead) {
					changeState(FollowingState.WALL_AHEAD);
				} else {
					//Tell Drive node to go forward, slowly
					cmd.data = DriveCommand.GO_STRAIGHT.name();
					msg.translationalVelocity = 0.2;
					drivePub.publish(msg);
				}
				break;
			case WALL_AHEAD:
				if (isWallAhead) {
					//Tell Drive node to turn 45 degrees to the right
					cmd.data = DriveCommand.TURN_FIXED_AMOUNT.name();
					msg.turnAngle = Math.PI/4;
					drivePub.publish(msg);
				} else {
					changeState(FollowingState.FOLLOW_WALL);
				}
				break;
			case FOLLOW_WALL:
				if (isWallSeen()) {
					//Tell Drive node to do a PID wall follow
					cmd.data = DriveCommand.FOLLOW_WALL.name();
					msg.translationalVelocity = 0.2;
					drivePub.publish(msg);
				} else { // We lost the wall
					changeState(FollowingState.FIND_WALL);
				}
				break;
			default:
				break;
		}
	}

	// Returns true iff there's a wall on left side, as indicated by sonar readings
	private boolean isWallSeen() {
		//TODO: implement
		return false;
	}

	private long getStateTime(){
		return System.currentTimeMillis() - stateStartTime;
	}

	private void changeState(FollowingState newState){
		stateStartTime = System.currentTimeMillis();
		state = newState;
	}

	private void publishState(){
		org.ros.message.std_msgs.String msg = new org.ros.message.std_msgs.String();
		msg.data = state.name();
		statePub.publish(msg);
	}

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
