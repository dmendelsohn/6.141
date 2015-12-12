package Drive;

import java.util.*;
import Challenge.*;

import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.*;
import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.parameter.*;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

public class SimpleDrive implements NodeMain {
	// Constants
	private static final boolean useSonarGUI = false; //TODO: make this an argument
	private final int NUM_BLOCKS_RETURN = 6;

	// State fields
	protected boolean firstUpdate = true;
	private double locX = 0;
	private double locY = 0;
	private double locTheta = 0;
	private List<Double> frontSonarBuffer;
	private List<Double> backSonarBuffer;
	private double frontSonar = -1;
	private double backSonar = -1;
	private boolean bumpLeft = false;
	private boolean bumpRight = false;
	private long stateBeginTime = 0;

	// Message State Fields
	private int blockFrameCount = 0;
	private double blockErrorX = 0;
	private double blockErrorY = 0;
	private double pathTrans = 0;
	private double pathRot = 0;
	private int blockCount = 1;

	// PID
	private PIDController anglePID;
	private PIDController forwardPID;

	// Sonar GUI
	private SonarGUI gui;

	// ROS Subscribers
	//public Subscriber<org.ros.message.rss_msgs.SonarMsg> frontSonarSub;
	//public Subscriber<org.ros.message.rss_msgs.SonarMsg> backSonarSub;
	//public Subscriber<org.ros.message.rss_msgs.BumpMsg> bumpSub;
	public Subscriber<org.ros.message.rss_msgs.OdometryMsg> localSub;
	public Subscriber<org.ros.message.rss_msgs.MotionMsg> pathFollowingSub;
	public Subscriber<org.ros.message.rss_msgs.ResetMsg> stopSub;
	public Subscriber<org.ros.message.rss_msgs.ResetMsg> newGoalSub;
	//public Subscriber<org.ros.message.rss_msgs.OdometryMsg> blockSub;

	// ROS Publishers
	public Publisher<org.ros.message.rss_msgs.MotionMsg> fixPub;
	public Publisher<org.ros.message.rss_msgs.ResetMsg> newGoalPub;
	public Publisher<org.ros.message.rss_msgs.ArmMsg> armPub;

	// Finite State Machine
	private int state;
	private int prev_state = 0;
	private final int FIND_NEXT_BLOCK = 0;
	private final int RETURN_AND_BUILD = 1;
	private final int VISUAL_SERVO = 2;
	private final int BACK_AWAY = 3;
	private final int REST = 4;
	private final int OVER_BLOCK = 5;

	public SimpleDrive() {
		if (useSonarGUI) {
			gui = new SonarGUI();
		} else {
			gui = null;
		}

		frontSonarBuffer = new LinkedList<Double>();
		backSonarBuffer = new LinkedList<Double>();

		anglePID = new PIDController(0.5, 0, 0, 0.5, 0.5, 0.5);
		forwardPID = new PIDController(0.5, 0, 0, 0.4, 0.4, 0.4);

		state = 0;
		changeState(FIND_NEXT_BLOCK);
	}

	@Override
	public void onStart(Node node) {
		//bumpSub = node.newSubscriber("/rss/BumpSensors", "rss_msgs/BumpMsg");
		//frontSonarSub = node.newSubscriber("/rss/Sonars/Front", "rss_msgs/SonarMsg");
		//backSonarSub = node.newSubscriber("/rss/Sonars/Back", "rss_msgs/SonarMsg");
		localSub = node.newSubscriber("/rss/localization", "rss_msgs/OdometryMsg");
		pathFollowingSub = node.newSubscriber("/command/PathFollowing", "rss_msgs/MotionMsg");
		stopSub = node.newSubscriber("/rss/stop", "rss_msgs/ResetMsg");
		newGoalSub = node.newSubscriber("/rss/newgoal", "rss_msgs/ResetMsg");
		//blockSub = node.newSubscriber("/rss/block", "rss_msgs/OdometryMsg");

		fixPub = node.newPublisher("command/Motors", "rss_msgs/MotionMsg");
		newGoalPub = node.newPublisher("/rss/newgoal", "rss_msgs/ResetMsg");
		armPub = node.newPublisher("command/Arm", "rss_msgs/ArmMsg");

		/*frontSonarSub.addMessageListener(new MessageListener<org.ros.message.rss_msgs.SonarMsg>() {
			@Override
			public void onNewMessage(org.ros.message.rss_msgs.SonarMsg message) {
				updateSonars(message.range, true);
			}
		});

		backSonarSub.addMessageListener(new MessageListener<org.ros.message.rss_msgs.SonarMsg>() {
			@Override
			public void onNewMessage(org.ros.message.rss_msgs.SonarMsg message) {
				updateSonars(message.range, false);
			}
		});

		bumpSub.addMessageListener(new MessageListener<org.ros.message.rss_msgs.BumpMsg>() {
			@Override
			public void onNewMessage(org.ros.message.rss_msgs.BumpMsg message) {
				handleBumpSensors(message);
			}
		});*/

		localSub.addMessageListener(new MessageListener<org.ros.message.rss_msgs.OdometryMsg>() {
			@Override
			public void onNewMessage(org.ros.message.rss_msgs.OdometryMsg message) {
				if (firstUpdate) {
					firstUpdate = false;
					if (useSonarGUI)
						gui.resetWorldToView(message.x, message.y);
				}
				if (useSonarGUI)
					gui.setRobotPose(message.x, message.y, message.theta);

				locX = message.x;
				locY = message.y;
				locTheta = message.theta;

				//System.out.println("SimpleDrive: localization is (x, y, theta) = " + locX + ", " + locY + ", " + locTheta);
			}
		});

		pathFollowingSub.addMessageListener(new MessageListener<org.ros.message.rss_msgs.MotionMsg>() {
			@Override
			public void onNewMessage(org.ros.message.rss_msgs.MotionMsg message) {
				pathTrans = message.translationalVelocity;
				pathRot = message.rotationalVelocity;

				handleState();
			}
		});

		stopSub.addMessageListener(new MessageListener<org.ros.message.rss_msgs.ResetMsg>() {
			@Override
			public void onNewMessage(org.ros.message.rss_msgs.ResetMsg message) {
				boolean go = message.reset;

				if (go && state == REST){
					changeState(prev_state);
				} else if (!go){
					changeState(REST);
				}
			}
		});

		newGoalSub.addMessageListener(new MessageListener<org.ros.message.rss_msgs.ResetMsg>() {
			@Override
			public void onNewMessage(org.ros.message.rss_msgs.ResetMsg message) {
				if (!message.reset){
					blockCount++;
					/*setVelocities(0.2, 0);
					try {
						Thread.sleep(100);
					} catch (Exception exc){
						exc.printStackTrace();
					}*/

					changeState(OVER_BLOCK);
					/*if (blockCount == NUM_BLOCKS_RETURN + 1){
						changeState(RETURN_AND_BUILD);
					} else if (blockCount > NUM_BLOCKS_RETURN + 1){
						changeState(BACK_AWAY);
					}

					if (blockCount <= NUM_BLOCKS_RETURN + 1){
						ResetMsg msg = new ResetMsg();
						msg.reset = true;
						newGoalPub.publish(msg);
					}*/
				}
			}
		});

		/*blockSub.addMessageListener(new MessageListener<org.ros.message.rss_msgs.OdometryMsg>() {
			@Override
			public void onNewMessage(org.ros.message.rss_msgs.OdometryMsg message) {
				if (message.theta > 0){
					blockFrameCount = 0;
				} else {
					//blockFrameCount++;
					blockErrorX = message.x;
					blockErrorY = message.y;

					if (blockFrameCount > 2){
						prev_state = state;
						state = VISUAL_SERVO;
					} else {
						state = prev_state;
					}
				}
			}
		});*/

		/*Thread controlThread = new Thread(new Runnable(){
			@Override
			public void run(){
				handleState();
				try{
					Thread.sleep(10);
				} catch (Exception exc){
					exc.printStackTrace();
				}
			}
		});
		controlThread.start();*/
	}

	private void handleState(){
		if (state == FIND_NEXT_BLOCK) {
			setVelocities(pathTrans, pathRot);
			//System.out.println("SimpleDrive: velocities (trans, rot) = " + pathTrans + ", " + pathRot);
		} else if (state == RETURN_AND_BUILD){
			setVelocities(pathTrans, pathRot);
			//System.out.println("SimpleDrive: velocities (trans, rot) = " + pathTrans + ", " + pathRot);
		} else if (state == BACK_AWAY){
			if (getStateTime() < 15000){
				System.out.println("SimpleDrive: turning");
				setVelocities(0, 0.2);
				setGate(false);
			} else if (getStateTime() < 25000){
				System.out.println("SimpleDrive: moving forward");
				setVelocities(0.2, 0);
				setGate(false);
			} else if (getStateTime() < 35000){
				System.out.println("SimpleDrive: moving forward");
				setVelocities(0.2, 0);
				setGate(true);
			} else {
				System.out.println("SimpleDrive: stopping");
				setVelocities(0, 0);
				setGate(false);
			}
		} else if (state == REST){
			setVelocities(0, 0);
		} else if (state == OVER_BLOCK){
			if (getStateTime() < 500){
				System.out.println("SimpleDrive: going over block");
				//setVelocities(0.1, 0);
				setVelocities(0, 0);
			} else {
				setVelocities(0, 0);
				if (blockCount == NUM_BLOCKS_RETURN + 1){
					changeState(RETURN_AND_BUILD);
				} else if (blockCount > NUM_BLOCKS_RETURN + 1){
					changeState(BACK_AWAY);
				} else {
					changeState(FIND_NEXT_BLOCK);
				}

				if (blockCount <= NUM_BLOCKS_RETURN + 1){
					ResetMsg msg = new ResetMsg();
					msg.reset = true;
					newGoalPub.publish(msg);
				}
			}
		}
		/*else if (state == VISUAL_SERVO){
			System.out.println("SimpleDrive: following block");
			setVelocities(forwardPID.update(blockErrorY), anglePID.update(blockErrorX));
		} */
		if (state != BACK_AWAY){
			setGate(false);
		}
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

	private void changeState(int newState){
		prev_state = state;
		state = newState;
		stateBeginTime = System.currentTimeMillis();
	}

	private long getStateTime(){
		return System.currentTimeMillis() - stateBeginTime;
	}

	private void setGate(boolean up){
		ArmMsg msg = new ArmMsg();
		long[] outpwms = new long[8];
		for (int i = 0; i < 8; i++){
			if (up){
				outpwms[i] = 800;
			} else {
				outpwms[i] = 1280;
			}
		}
		msg.pwms = outpwms;
		armPub.publish(msg);
	}

	private void setVelocities(double trans, double rot){
		org.ros.message.rss_msgs.MotionMsg msg = new org.ros.message.rss_msgs.MotionMsg();
		msg.translationalVelocity = trans;
		msg.rotationalVelocity = rot;
		fixPub.publish(msg);
	}

	private void handleBumpSensors(org.ros.message.rss_msgs.BumpMsg message){
		bumpLeft = message.left;
		bumpRight = message.right;
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
		return new GraphName("rss/simpledrive");
	}
}
