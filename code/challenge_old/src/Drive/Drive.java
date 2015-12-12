package Drive;

import java.util.*;
import Challenge.*;

import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.*;
import org.ros.message.challenge_msgs.*;
import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.parameter.*;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

//TODO: implement all drive commands, command handling
//TODO: change terminology to use "Wall follow" everywhere, not "Follow wall", "wall following" or anything else
public class Drive implements NodeMain {
	// Constants
	public enum DriveCommand {GO_STRAIGHT, TURN_FIXED_AMOUNT, FOLLOW_WALL, STOP};
	public enum DriveState {CONSTANT, TURN_FIXED_AMOUNT, FOLLOW_WALL};
	public enum Driver {MAIN_FSM, WALL_FOLLOWING_FSM, VISUAL_SERVO};
	public enum RunMode { // This is used as a runtime argument that allows this node to behave in different ways
		WALL_FOLLOW_ONLY, //only listens to Drive commands from WallFollowingFSM
		VISUAL_SERVO_ONLY, //only listens to Drive commands from VisualServo (Not yet implemented)
		MAIN_FSM_ONLY, //only listen to Drive commands from the main FSM
		MANUAL, //Listens to no drive commands, control via keyboard commands (Not yet implemented)
		NORMAL //Listens to FSM, FSM instructs this node begin/stop listening to WallFollowingFSM
	};

	// Constants
	public static final String WALL_FOLLOWING_FSM_TOPIC = "/rss/Drive/WallFollowing";
	public static final String MAIN_FSM_TOPIC = "/rss/Drive/FSM";
	public static final String VISUAL_SERVO_TOPIC = "/rss/Drive/VisualServo";

	// State
	private DriveState driveState;
	private Driver driver;
	private RunMode runMode;
	private boolean useGUI;

	// State fields
	private boolean firstUpdate = true;
	private double locX = 0;
	private double locY = 0;
	private double locTheta = 0;
	private SonarBuffer sonarBuffer;
	private boolean bumpLeft = false;
	private boolean bumpRight = false;

	// Other instance variables
	private SonarGUI gui;

	// ROS Subscribers
	public Subscriber<org.ros.message.challenge_msgs.DriveMsg> wallFollowSub;
	public Subscriber<org.ros.message.challenge_msgs.DriveMsg> fsmSub;
	public Subscriber<org.ros.message.challenge_msgs.DriveMsg> visualServoSub;
	public Subscriber<org.ros.message.rss_msgs.SonarMsg> frontSonarSub;
	public Subscriber<org.ros.message.rss_msgs.SonarMsg> backSonarSub;
	public Subscriber<org.ros.message.rss_msgs.BumpMsg> bumpSub;
	public Subscriber<org.ros.message.rss_msgs.OdometryMsg> odoSub;

	// ROS Publishers
	public Publisher<org.ros.message.rss_msgs.MotionMsg> fixPub;

	public Drive() {
		gui = null;
		driveState = null;
		sonarBuffer = new SonarBuffer();
	}

	@Override
	public void onStart(Node node) {
		ParameterTree paramTree = node.newParameterTree();
		String useGUIString = paramTree.getString(node.resolveName("~/useGUI"));

		//Set up whether or not to use sthe SonarGUI
		this.useGUI = Boolean.parseBoolean(useGUIString);
		if (this.useGUI) {
			gui = new SonarGUI();
		}

		//Get runMode and determine initial settings for driver and driveState
		String runModeString = paramTree.getString(node.resolveName("~/runMode"));
		try {
			this.runMode = RunMode.valueOf(runModeString);
		} catch (IllegalArgumentException e) {
			e.printStackTrace();
			this.runMode = RunMode.NORMAL;
		}
		driveState = DriveState.CONSTANT; //Don't move until a command indicates otherwise
		switch(runMode) {
			case WALL_FOLLOW_ONLY:
				driver = Driver.WALL_FOLLOWING_FSM; //In this case, this won't ever change
				break;
			case VISUAL_SERVO_ONLY:
				driver = Driver.VISUAL_SERVO; //In this case, this won't ever change
				break;
			default:
				driver = Driver.MAIN_FSM; //This is the typical case
		}   

		// Initialize subscribers
		wallFollowSub = node.newSubscriber(WALL_FOLLOWING_FSM_TOPIC, "challenge_msgs/DriveMsg");
		fsmSub = node.newSubscriber(MAIN_FSM_TOPIC, "challenge_msgs/DriveMsg");
		visualServoSub = node.newSubscriber(VISUAL_SERVO_TOPIC, "challenge_msgs/DriveMsg");
		bumpSub = node.newSubscriber("/rss/BumpSensors", "rss_msgs/BumpMsg");
		frontSonarSub = node.newSubscriber("/rss/Sonars/Front", "rss_msgs/SonarMsg");
		backSonarSub = node.newSubscriber("/rss/Sonars/Back", "rss_msgs/SonarMsg");
		odoSub = node.newSubscriber("/rss/odometry", "rss_msgs/OdometryMsg");
		
		fsmSub.addMessageListener(new MessageListener<org.ros.message.challenge_msgs.DriveMsg>() {
			@Override
			public void onNewMessage(org.ros.message.challenge_msgs.DriveMsg message) {
				handleDriveMessage(Driver.MAIN_FSM, message);
			}
		});

		wallFollowSub.addMessageListener(new MessageListener<org.ros.message.challenge_msgs.DriveMsg>() {
			@Override
			public void onNewMessage(org.ros.message.challenge_msgs.DriveMsg message) {
				handleDriveMessage(Driver.WALL_FOLLOWING_FSM, message);
			}
		});


		visualServoSub.addMessageListener(new MessageListener<org.ros.message.challenge_msgs.DriveMsg>() {
			@Override
			public void onNewMessage(org.ros.message.challenge_msgs.DriveMsg message) {
				handleDriveMessage(Driver.VISUAL_SERVO, message);
			}
		});

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
				if (useGUI) {
					if (firstUpdate) {
						firstUpdate = false;
						gui.resetWorldToView(message.x, message.y);
					}
					gui.setRobotPose(message.x, message.y, message.theta);
				}
				locX = message.x;
				locY = message.y;
				locTheta = message.theta;
			}
		});
	}

	private void handleDriveMessage(Driver caller, org.ros.message.challenge_msgs.DriveMsg message) {
		//TODO: implement
	}

	private void setVelocities(double trans, double rot){
		org.ros.message.rss_msgs.MotionMsg msg = new org.ros.message.rss_msgs.MotionMsg();
		msg.translationalVelocity = trans;
		msg.rotationalVelocity = rot;
		fixPub.publish(msg);
	}

	private void handleState(){
		// fill in with state drive logic
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
		return new GraphName("rss/localnavigation");
	}
}
