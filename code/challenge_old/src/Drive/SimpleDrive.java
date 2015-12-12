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

public class SimpleDrive implements NodeMain {
	// State fields
	protected boolean firstUpdate = true;
	private boolean go = true;
	private double locX = 0;
	private double locY = 0;
	private double locTheta = 0;
	private List<Double> frontSonarBuffer;
	private List<Double> backSonarBuffer;
	private double frontSonar = -1;
	private double backSonar = -1;
	private boolean bumpLeft = false;
	private boolean bumpRight = false;

	// State
	private int state;
	private final int NO_STATE = -1;

	// Sonar GUI
	private SonarGUI gui;

	// ROS Subscribers
	public Subscriber<org.ros.message.rss_msgs.SonarMsg> frontSonarSub;
	public Subscriber<org.ros.message.rss_msgs.SonarMsg> backSonarSub;
	public Subscriber<org.ros.message.rss_msgs.BumpMsg> bumpSub;
	public Subscriber<org.ros.message.rss_msgs.OdometryMsg> localSub;
	public Subscriber<org.ros.message.rss_msgs.MotionMsg> pathFollowingSub;
	public Subscriber<org.ros.message.rss_msgs.ResetMsg> stopSub;

	// ROS Publishers
	public Publisher<org.ros.message.rss_msgs.MotionMsg> fixPub;

	public SimpleDrive() {
		gui = new SonarGUI();

		frontSonarBuffer = new LinkedList<Double>();
		backSonarBuffer = new LinkedList<Double>();
	}

	@Override
	public void onStart(Node node) {
		bumpSub = node.newSubscriber("/rss/BumpSensors", "rss_msgs/BumpMsg");
		frontSonarSub = node.newSubscriber("/rss/Sonars/Front", "rss_msgs/SonarMsg");
		backSonarSub = node.newSubscriber("/rss/Sonars/Back", "rss_msgs/SonarMsg");
		localSub = node.newSubscriber("/rss/localization", "rss_msgs/OdometryMsg");
		pathFollowingSub = node.newSubscriber("/command/PathFollowing", "rss_msgs/MotionMsg");
		stopSub = node.newSubscriber("/rss/stop", "rss_msgs/ResetMsg");

		fixPub = node.newPublisher("command/Motors", "rss_msgs/MotionMsg");

		frontSonarSub.addMessageListener(new MessageListener<org.ros.message.rss_msgs.SonarMsg>() {
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
		});

		localSub.addMessageListener(new MessageListener<org.ros.message.rss_msgs.OdometryMsg>() {
			@Override
			public void onNewMessage(org.ros.message.rss_msgs.OdometryMsg message) {
				if (firstUpdate) {
					firstUpdate = false;
					gui.resetWorldToView(message.x, message.y);
				}
				gui.setRobotPose(message.x, message.y, message.theta);

				locX = message.x;
				locY = message.y;
				locTheta = message.theta;
			}
		});

		pathFollowingSub.addMessageListener(new MessageListener<org.ros.message.rss_msgs.MotionMsg>() {
			@Override
			public void onNewMessage(org.ros.message.rss_msgs.MotionMsg message) {
				if (go){
					double rot = message.translationalVelocity;
					double trans = message.rotationalVelocity;
					setVelocities(rot, trans);
				} else {
					setVelocities(0, 0);
				}
			}
		});

		stopSub.addMessageListener(new MessageListener<org.ros.message.rss_msgs.ResetMsg>() {
			@Override
			public void onNewMessage(org.ros.message.rss_msgs.ResetMsg message) {
				go = message.reset;
			}
		});
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
		return new GraphName("rss/localnavigation");
	}
}
