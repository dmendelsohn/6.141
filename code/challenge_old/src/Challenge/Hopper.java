package Challenge;

import java.util.concurrent.ArrayBlockingQueue;

import java.awt.Color;
import java.awt.geom.*;
import java.util.*;

import org.ros.namespace.GraphName;
import org.ros.node.parameter.*;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.*;

import Gripper.*;
import Vision.*;
import Challenge.*;

public class Hopper implements NodeMain {
	// ROS Subscribers
	private Subscriber<org.ros.message.rss_msgs.ArmMsg> armSub;
	private Subscriber<org.ros.message.rss_msgs.BumpMsg> bumpSub;
	private Subscriber<org.ros.message.rss_msgs.ResetMsg> imageStatsSub;

	// ROS Publishers
	private Publisher<org.ros.message.rss_msgs.ArmMsg> armPub;
	public Publisher<org.ros.message.rss_msgs.MotionMsg> motionPub;

	// Map GUI
	ArmPoseGUI gui;

	// Constants
	private final double ARM_LENGTH = 0.27; // TO-DO Length from shoulder to wrist in meters
	private final double WRIST_LENGTH = 0.12; // TO-DO Length from wrist to bump-tip(end-effecter)
	//private final double SHOPOSX; // TO-DO  X position of shoulder joint in meters
	//private final double SHOPOSZ; // TO-DO  Z position of shoulder joint in meters
	private final double THETA_GAIN = 0.5;

	private final long SHOULDER_MIN = 700;
	private final long SHOULDER_MAX = 2400;
	private final long SHOULDER_SEP = 1500;
	private final long SHOULDER_FLAT = 1550;

	private final long WRIST_MIN = 575;
	private final long WRIST_MAX = 2250;
	private final long WRIST_SEP = 1650;
	private final long WRIST_FLAT = 1400;

	//private final long GRIP_OPEN = 1900;
	private final long GRIP_OPEN = 2000;
	private final long GRIP_CLOSED = 1650;

	private final double THETA_TOL = 0.05;
	private final double MAX_THETA_DIFF = 0.5;

	private final double ROTATION_GAIN = 0.2;

	// Variables
	private double goalX; // distance from origin to object along the X-axis
	private double goalZ; // distance from origin to object along the Z-axis

	private double shoulderTheta;
	private double wristTheta;
	private double gripTheta;

	private int gymnasticsIndex = 0;
	private int graspIndex = 0;

	private long lagTime = 0;
	private long moveTime = 0;

	private boolean objectDetected = false;

	// Vision
	//Publisher<org.ros.message.sensor_msgs.Image> vidPub;
	//Subscriber<org.ros.message.sensor_msgs.Image> vidSub;

	private BlobTracking blobTrack;

	private static final int width = 160;
	private static final int height = 120;

	private double visualError = 1000.0;
	private boolean beginRoutine = false;


	public Hopper(){
		//gui = new ArmPoseGUI();

		goalX = 0.32;
		goalZ = 0;

		shoulderTheta = 0;
		wristTheta = 0;
		gripTheta = 0;

		blobTrack = new BlobTracking(width, height);
	}

	/**
	 * Entry hook for ROS when called as stand-alone node
	 */
	@Override
	public void onStart(Node node) {
		armSub = node.newSubscriber("rss/ArmStatus", "rss_msgs/ArmMsg");
		bumpSub = node.newSubscriber("rss/BumpSensors", "rss_msgs/BumpMsg");
		//vidSub = node.newSubscriber("rss/video", "sensor_msgs/Image");
		imageStatsSub = node.newSubscriber("rss/ImageStats", "rss_msgs/ResetMsg");

		armSub.addMessageListener(new MessageListener<org.ros.message.rss_msgs.ArmMsg>() {
			@Override
			public void onNewMessage(org.ros.message.rss_msgs.ArmMsg message) {
				handle(message);
			}
		});
		bumpSub.addMessageListener(new MessageListener<org.ros.message.rss_msgs.BumpMsg>(){
			@Override
			public void onNewMessage(org.ros.message.rss_msgs.BumpMsg message){
				handle(message);
			}
		});
		/*
		vidSub.addMessageListener(new MessageListener<org.ros.message.sensor_msgs.Image>() {
			@Override
			public void onNewMessage(org.ros.message.sensor_msgs.Image message) {
				byte[] rgbData;
				boolean reverseRGB = false;

				if (reverseRGB) {
					rgbData = Image.RGB2BGR(message.data,
							(int) message.width, (int) message.height);
				}
				else {
					rgbData = message.data;
				}
				assert ((int) message.width == width);
				assert ((int) message.height == height);

				Image src = null;
				try {
					src = new Image(rgbData, width, height);
				} catch (Exception e) {
					e.printStackTrace();
					return;
				}
			
				Image dest = new Image(src);
			
				blobTrack.apply(src, dest);

				//visualError = 0.5 - (((double) blobTrack.centroidX) / width);
				//System.out.println("VISUAL ERROR OBTAINED: " + visualError);

				org.ros.message.sensor_msgs.Image pubImage = new org.ros.message.sensor_msgs.Image();
  				pubImage.width = width;
  				pubImage.height = height;
  				pubImage.encoding = "rgb8";
  				pubImage.is_bigendian = 0;
  				pubImage.step = width*3;
 				pubImage.data = dest.toArray();
  				vidPub.publish(pubImage);
			}
		});
		*/
		imageStatsSub.addMessageListener(new MessageListener<org.ros.message.rss_msgs.ResetMsg>(){
			@Override
			public void onNewMessage(org.ros.message.rss_msgs.ResetMsg message){
				beginRoutine = !message.reset;
			}
		});

		armPub = node.newPublisher("command/Arm", "rss_msgs/ArmMsg");
		motionPub = node.newPublisher("command/Motors", "rss_msgs/MotionMsg");
		//vidPub = node.newPublisher("/rss/blobVideo", "sensor_msgs/Image");
	}

	private void handle(ArmMsg msg) {
		handleKinematics(msg);
		//handleGymnastics(msg);
		System.out.println("Begin Routine: " + beginRoutine);
		//handleGrasping(msg);
	}

	private void handleGymnastics(ArmMsg msg){
		// arm gymnastics
		int numRoutines = 10;
		boolean doneTask = false;

		if (gymnasticsIndex == 0){
			doneTask = moveShoulder(msg, Math.PI/8);
		} else if (gymnasticsIndex == 1){
			doneTask = openGripper(msg);
		} else if (gymnasticsIndex == 2){
			doneTask = bendElbow(msg, Math.PI/2);
		} else if (gymnasticsIndex == 3){
			doneTask = closeGripper(msg);
		} else if (gymnasticsIndex == 4){
			doneTask = moveShoulder(msg, -Math.PI/8);
		} else if (gymnasticsIndex == 5){
			doneTask = openGripper(msg);
		} else if (gymnasticsIndex == 6){
			doneTask = bendElbow(msg, 0.0);
		} else if (gymnasticsIndex == 7){
			doneTask = closeGripper(msg);
		} else if (gymnasticsIndex == 8){
			doneTask = bendElbow(msg, Math.PI/2);
		} else if (gymnasticsIndex == 9){
			doneTask = moveToGround(msg);
		}

		System.out.println("Task Index: " + gymnasticsIndex);

		if (doneTask){
			if (lagTime <= 0){
				lagTime = System.currentTimeMillis();
			} else if ((System.currentTimeMillis() - lagTime > 500 && gymnasticsIndex != 8) ||
				(System.currentTimeMillis() - lagTime > 1500)){
				gymnasticsIndex++;
				if (gymnasticsIndex >= numRoutines){
					gymnasticsIndex = 0;
				}
				lagTime = 0;
			}
		}
	}

	private void handleGrasping(ArmMsg msg) {
		// grasping functionality
		int numRoutines = 9;
		boolean doneTask = false;

		if (graspIndex == 0){
			doneTask = moveToGround(msg);
		} else if (graspIndex == 1){
			//doneTask = faceBlock();
			doneTask = true;
		} else if (graspIndex == 2){
			//doneTask = moveToGround(msg);
			doneTask = obtainBlock(msg, 0.15);
		} else if (graspIndex == 3){
			doneTask = closeGripper(msg);
			setVelocities(0, 0);
		} else if (graspIndex == 4){
			//doneTask = moveShoulder(msg, -Math.PI/4);
			doneTask = moveShoulder(msg, -Math.PI/4);
			setVelocities(0, 0);
		} else if (graspIndex == 5){
			doneTask = bendElbow(msg, Math.PI/4);
			setVelocities(0, 0);
		} else if (graspIndex == 6){
			doneTask = moveForward(4000, 0.3);
		} else if (graspIndex == 7){
			// moveToGround also opens the gripper.
			doneTask = moveToGround(msg);
			setVelocities(0, 0);
		} else if (graspIndex == 8){
			doneTask = moveForward(4000, -0.3);
		}

		/*if (objectDetected) {
			if (graspIndex == 1){
				doneTask = closeGripper(msg);
			} else if (graspIndex == 2){
				doneTask = moveShoulder(msg, -Math.PI/4);
			} else if (graspIndex == 3){
				doneTask = bendElbow(msg, Math.PI/4);
			} else if (graspIndex == 4){
				doneTask = true;
				// (move robot)
			} else if (graspIndex == 5){
				// moveToGround also opens the gripper.
				doneTask = moveToGround(msg);
			}
		} else {
			graspIndex = 0;
			// moveToGround also opens the gripper.
			doneTask = moveToGround(msg);
		}*/

		if (doneTask){
			if (lagTime <= 0){
				lagTime = System.currentTimeMillis();
			} else if (System.currentTimeMillis() - lagTime > 500){
				if (!objectDetected && graspIndex < 7 && graspIndex > 1){
					graspIndex = 2;
				} else {
					if (beginRoutine || graspIndex == 0){
						graspIndex++;
						if (graspIndex >= numRoutines){
							graspIndex = 0;
						}
						lagTime = 0;
					}
				}
			}
		}

		System.out.println("Object Detected: " + objectDetected);
		System.out.println("Grasp Index: " + graspIndex);
	}

	private boolean faceBlock(){
		/*System.out.println("Block Center, Width, Visual Error: " + blobTrack.centroidX + ", " + width + ", " + visualError);

		if (blobTrack.centroidX == 0){
			setVelocities(0, 0);
		} else if (Math.abs(visualError) < 2){
			setVelocities(0, ROTATION_GAIN * visualError);
		}

		if (Math.abs(visualError) < 0.02){
			return true;
		} else {
			return false;
		}*/

		//System.out.println("Curr X, Goal X, rot: " + blobTrack.centroidX + ", " + (width / 2.0) + ", " + blobTrack.rotationVelocityCommand);

		/*setVelocities(0, blobTrack.rotationVelocityCommand);

		if (Math.abs(blobTrack.rotationVelocityCommand) < 0.05){
			return true;
		} else {
			return false;
		}*/
		return false;
	}

	private boolean obtainBlock(ArmMsg msg, double velocity){
		boolean doneArm = moveToGround(msg);

		if (doneArm){
			setVelocities(velocity, 0.0);
		}

		return objectDetected;
	}

	private boolean moveForward(long time, double velocity){
		if (moveTime <= 0){
			moveTime = System.currentTimeMillis();
		}

		setVelocities(velocity, 0.0);

		if (System.currentTimeMillis() > moveTime + time){
			moveTime = 0;
			setVelocities(0.0, 0.0);
			return true;
		} else {
			return false;
		}
	}

	private void setVelocities(double trans, double rot){
		org.ros.message.rss_msgs.MotionMsg msg = new org.ros.message.rss_msgs.MotionMsg();
		msg.translationalVelocity = trans;
		msg.rotationalVelocity = rot;
		motionPub.publish(msg);
	}

	private boolean openGripper(ArmMsg msg){
		long shoulderPWM = msg.pwms[0];
		long wristPWM = msg.pwms[1];
		long gripPWM = msg.pwms[2];
		if (shoulderPWM < SHOULDER_MIN){
			shoulderPWM = (SHOULDER_MIN + SHOULDER_MAX) / 2;
		}
		if (wristPWM < WRIST_MIN){
			wristPWM = (WRIST_MIN + WRIST_MAX) / 2;
		}

		setPWMs(shoulderPWM, wristPWM, GRIP_OPEN);

		if (Math.abs(gripPWM - GRIP_OPEN) < THETA_TOL){
			return true;
		} else {
			return false;
		}
	}

	private boolean closeGripper(ArmMsg msg){
		long shoulderPWM = msg.pwms[0];
		long wristPWM = msg.pwms[1];
		long gripPWM = msg.pwms[2];
		if (shoulderPWM < SHOULDER_MIN){
			shoulderPWM = (SHOULDER_MIN + SHOULDER_MAX) / 2;
		}
		if (wristPWM < WRIST_MIN){
			wristPWM = (WRIST_MIN + WRIST_MAX) / 2;
		}

		setPWMs(shoulderPWM, wristPWM, GRIP_CLOSED);

		if (Math.abs(gripPWM - GRIP_CLOSED) < THETA_TOL){
			return true;
		} else {
			return false;
		}
	}

	private boolean moveShoulder(ArmMsg msg, double theta){
		long shoulderPWM = msg.pwms[0];
		long wristPWM = msg.pwms[1];
		long gripPWM = msg.pwms[2];
		if (shoulderPWM < SHOULDER_MIN){
			shoulderPWM = (SHOULDER_MIN + SHOULDER_MAX) / 2;
		}
		if (wristPWM < WRIST_MIN){
			wristPWM = (WRIST_MIN + WRIST_MAX) / 2;
		}
		double[] currentThetas = convertPWMToThetas(shoulderPWM, wristPWM, gripPWM);

		double shoulderError = theta - currentThetas[0];

		//shoulderTheta = Math.max(Math.min(shoulderTheta + THETA_GAIN * shoulderError, currentThetas[0] + MAX_THETA_DIFF), currentThetas[0] - MAX_THETA_DIFF);

		shoulderTheta = Math.max(Math.min(theta, currentThetas[0] + MAX_THETA_DIFF), currentThetas[0] - MAX_THETA_DIFF);

		long[] outPWM = convertThetasToPWM(shoulderTheta, 0, 0);
		setPWMs(outPWM[0], wristPWM, gripPWM);

		if (Math.abs(shoulderError) < THETA_TOL){
			return true;
		} else {
			return false;
		}
	}

	private boolean bendElbow(ArmMsg msg, double theta){
		long shoulderPWM = msg.pwms[0];
		long wristPWM = msg.pwms[1];
		long gripPWM = msg.pwms[2];
		if (shoulderPWM < SHOULDER_MIN){
			shoulderPWM = (SHOULDER_MIN + SHOULDER_MAX) / 2;
		}
		if (wristPWM < WRIST_MIN){
			wristPWM = (WRIST_MIN + WRIST_MAX) / 2;
		}
		double[] currentThetas = convertPWMToThetas(shoulderPWM, wristPWM, gripPWM);

		double wristError = theta - currentThetas[1];

		//wristTheta = Math.max(Math.min(wristTheta + THETA_GAIN * wristError, currentThetas[1] + MAX_THETA_DIFF), currentThetas[1] - MAX_THETA_DIFF);

		wristTheta = Math.max(Math.min(theta, currentThetas[1] + MAX_THETA_DIFF), currentThetas[1] - MAX_THETA_DIFF);

		long[] outPWM = convertThetasToPWM(0, wristTheta, 0);
		setPWMs(shoulderPWM, outPWM[1], gripPWM);

		if (Math.abs(wristError) < THETA_TOL){
			return true;
		} else {
			return false;
		}
	}

	private boolean moveToGround(ArmMsg msg){
		long shoulderPWM = msg.pwms[0];
		long wristPWM = msg.pwms[1];
		long gripPWM = msg.pwms[2];
		if (shoulderPWM < SHOULDER_MIN){
			shoulderPWM = (SHOULDER_MIN + SHOULDER_MAX) / 2;
		}
		if (wristPWM < WRIST_MIN){
			wristPWM = (WRIST_MIN + WRIST_MAX) / 2;
		}
		double[] currentThetas = convertPWMToThetas(shoulderPWM, wristPWM, gripPWM);

		double[] goalThetas = new double[]{-0.9 * Math.PI / 2, 0.9 * Math.PI / 2 };
		double shoulderError = goalThetas[0] - currentThetas[0];
		double wristError = goalThetas[1] - currentThetas[1];

		//shoulderTheta = Math.max(Math.min(shoulderTheta + THETA_GAIN * shoulderError, currentThetas[0] + MAX_THETA_DIFF), currentThetas[0] - MAX_THETA_DIFF);
		//wristTheta = Math.max(Math.min(wristTheta + THETA_GAIN * wristError, currentThetas[1] + MAX_THETA_DIFF), currentThetas[1] - MAX_THETA_DIFF);

		shoulderTheta = Math.max(Math.min(goalThetas[0], currentThetas[0] + MAX_THETA_DIFF), currentThetas[0] - MAX_THETA_DIFF);
		wristTheta = Math.max(Math.min(goalThetas[1], currentThetas[1] + MAX_THETA_DIFF), currentThetas[1] - MAX_THETA_DIFF);

		long[] outPWM = convertThetasToPWM(shoulderTheta, wristTheta, GRIP_OPEN);
		setPWMs(outPWM[0], outPWM[1], GRIP_OPEN);

		if (Math.abs(shoulderError) < THETA_TOL && Math.abs(wristError) < THETA_TOL){
			return true;
		} else {
			return false;
		}
	}

	private void handleKinematics(ArmMsg msg){
		// get current thetas
		long shoulderPWM = msg.pwms[0];
		long wristPWM = msg.pwms[1];
		long gripPWM = msg.pwms[2];
		if (shoulderPWM < SHOULDER_MIN){
			shoulderPWM = (SHOULDER_MIN + SHOULDER_MAX) / 2;
		}
		if (wristPWM < WRIST_MIN){
			wristPWM = (WRIST_MIN + WRIST_MAX) / 2;
		}
		double[] currentThetas = convertPWMToThetas(shoulderPWM, wristPWM, gripPWM);

		// find goal thetas and apply P control to find output thetas
		double[] goalThetas = inverseKinematics(goalX, goalZ);
		double shoulderError = goalThetas[0] - currentThetas[0];
		double wristError = goalThetas[1] - currentThetas[1];

		shoulderTheta = Math.max(Math.min(goalThetas[0], currentThetas[0] + MAX_THETA_DIFF), currentThetas[0] - MAX_THETA_DIFF);
		wristTheta = Math.max(Math.min(goalThetas[1], currentThetas[1] + MAX_THETA_DIFF), currentThetas[1] - MAX_THETA_DIFF);

		//shoulderTheta = Math.max(Math.min(shoulderTheta + THETA_GAIN * shoulderError, currentThetas[0] + MAX_THETA_DIFF), currentThetas[0] - MAX_THETA_DIFF);
		//wristTheta = Math.max(Math.min(wristTheta + THETA_GAIN * wristError, currentThetas[1] + MAX_THETA_DIFF), currentThetas[1] - MAX_THETA_DIFF);

		// change this to handle grip PWM as well
		long[] outPWM = convertThetasToPWM(shoulderTheta, wristTheta, 0);
		setPWMs(outPWM[0], outPWM[1], 0);

		//long[] outPWM = convertThetasToPWM(0.0, 0.0, 0.0);
		//setPWMs(outPWM[0], outPWM[1], 0);

		System.out.println("Current PWMs: " + shoulderPWM + ", " + wristPWM);
		System.out.println("Current Thetas: " + currentThetas[0] + ", " + currentThetas[1]);
		System.out.println("Goal Thetas: " + goalThetas[0] + ", " + goalThetas[1]);
		System.out.println("Output PWMs: " + outPWM[0] + ", " + outPWM[1]);
		System.out.println("Output Thetas: " + shoulderTheta + ", " + wristTheta);
	}

	private void handle(BumpMsg msg){
		if (msg.gripper == true) {
			objectDetected = true;
		} else {
			objectDetected = false;
		}
	}

	public void setPWMs(long shoulderPWM, long wristPWM, long gripPWM){
		org.ros.message.rss_msgs.ArmMsg msg = new org.ros.message.rss_msgs.ArmMsg();
		long[] PWMs = new long[]{0,0,0,0,0,0,0,0};
		PWMs[0] = Math.max(Math.min(SHOULDER_MAX, shoulderPWM), SHOULDER_MIN);
		PWMs[1] = Math.max(Math.min(WRIST_MAX, wristPWM), WRIST_MIN);
		PWMs[2] = Math.max(Math.min(2100, gripPWM), 0);
		msg.pwms = PWMs;
		armPub.publish(msg);
	}

	private long[] convertThetasToPWM(double shoulderTheta, double wristTheta, double gripTheta){
		long[] PWM = new long[2];

		PWM[0] = (long) ((SHOULDER_SEP * shoulderTheta / Math.PI) + SHOULDER_FLAT);
		PWM[1] = (long) (WRIST_FLAT - (WRIST_SEP * wristTheta / Math.PI));

		return PWM;
	}

	private double[] convertPWMToThetas(long shoulderPWM, long wristPWM, long gripPWM){
		double[] thetas = new double[2];

		thetas[0] = Math.PI * (shoulderPWM - SHOULDER_FLAT) / SHOULDER_SEP;
		thetas[1] = Math.PI * (WRIST_FLAT - wristPWM) / WRIST_SEP;

		//thetas[0] = Math.PI * (1400.0 / 1650.0) - shoulderPWM * (Math.PI / 1650.0);
		//thetas[1] = Math.PI * (675.0 + 2300.0) / (2.0 * 1625.0) - wristPWM * (Math.PI / 1625.0);

		for (int i = 0; i < 2; i++){
			if (thetas[i] < -Math.PI){
				thetas[i] += 2 * Math.PI;
			} else if (thetas[i] >= Math.PI){
				thetas[i] -= 2 * Math.PI;
			}
		}

		if (Math.abs(thetas[0]) > Math.PI || Math.abs(thetas[1]) > Math.PI){
			System.out.println("THETAS OUT OF RANGE");
		}

		return thetas;
	}

	private double[] inverseKinematics(double x, double z){
		double angleToPoint = Math.atan(z/x);
		double angleToWristJoint = Math.acos((Math.pow(x, 2.0) + Math.pow(z, 2.0) + Math.pow(ARM_LENGTH, 2.0) - Math.pow(WRIST_LENGTH, 2.0))/
				(2.0*ARM_LENGTH*Math.sqrt(Math.pow(x, 2.0) + Math.pow(z, 2.0))));

		double internalWristAngle = Math.acos((Math.pow(ARM_LENGTH, 2.0) + Math.pow(WRIST_LENGTH, 2.0) - Math.pow(x, 2.0) - Math.pow(z, 2.0))/
				(2.0*ARM_LENGTH*WRIST_LENGTH));

		double[] thetas = new double[2];
		thetas[0] = angleToPoint - angleToWristJoint;
		thetas[1] = Math.PI - internalWristAngle;

		for (int i = 0; i < 2; i++){
			if (thetas[i] < -Math.PI){
				thetas[i] += 2 * Math.PI;
			} else if (thetas[i] >= Math.PI){
				thetas[i] -= 2 * Math.PI;
			}
		}

		return thetas;
	}

	/*
	//set goal point
	public void setGoalPoint(double x, double z){
		goalX = new Double(x - SHOPOSX);
		goalZ = new Double(z - SHOPOSZ);
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
		return new GraphName("rss/grasping");
	}

}
