package MotionPlanning;

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
import org.ros.message.lab5_msgs.*;
import org.ros.message.lab6_msgs.*;

import Drive.*;

public class PathFollower implements NodeMain {
	// ROS subscribers
	public Subscriber<org.ros.message.rss_msgs.OdometryMsg> localSub;
	public Subscriber<org.ros.message.lab6_msgs.GUIPolyMsg> pathSub;

	// ROS publishers
	public Publisher<org.ros.message.rss_msgs.MotionMsg> motionPub;

	// State Variables
	private double locX = 0;
	private double locY = 0;
	private double locTheta = 0;
	private double goalTheta = 0;
	private List<Point2D.Double> goalPath;
	private int pathIndex = 0;
	private double goalX = 0;
	private double goalY = 0;
	boolean done = false;
	private PIDController anglePID;

	public PathFollower() {
		goalPath = new LinkedList<Point2D.Double>();
		anglePID = new PIDController(0.15, 0, 0, 2, 2, 2);
	}

	/**
	 * Entry hook for ROS when called as stand-alone node
	 */
	@Override
	public void onStart(Node node) {
		System.out.println("REACHED ROBOT NAVIGATION");

		motionPub = node.newPublisher("/command/PathFollowing", "rss_msgs/MotionMsg");
		localSub = node.newSubscriber("/rss/localization", "rss_msgs/OdometryMsg");
		pathSub = node.newSubscriber("/command/path", "lab6_msgs/GUIPolyMsg");

		pathSub.addMessageListener(new MessageListener<org.ros.message.lab6_msgs.GUIPolyMsg>() {
			@Override
			public void onNewMessage(org.ros.message.lab6_msgs.GUIPolyMsg message) {
				for (int i = 0; i < message.numVertices; i++){
					Point2D.Double point = new Point2D.Double(message.x[i], message.y[i]);
					goalPath.add(point);
				}
			}
		});

		localSub.addMessageListener(new MessageListener<org.ros.message.rss_msgs.OdometryMsg>() {
			@Override
			public void onNewMessage(org.ros.message.rss_msgs.OdometryMsg message) {
				System.out.println("Getting Odometry");
				locX = message.x;
				locY = message.y;
				locTheta = message.theta;

				navigateRobot();
			}
		});
	}

	public void navigateRobot(){
		System.out.println("Path Index: " + pathIndex);
		System.out.println("goalPath Size: " + goalPath.size());
		System.out.println("locX, locY, locTheta" + locX + ", " + locY + ", " + locTheta);
		System.out.println("goalX, goalY, goalTheta: " + goalX + ", " + goalY + ", " + goalTheta);
		if (goalPath.size() > 0 && !done){
			if (pathIndex < goalPath.size() - 1 &&
				Math.abs(goalX - locX) < 0.2 &&
				Math.abs(goalY - locY) < 0.2){
				pathIndex++;
			}
			if (pathIndex == goalPath.size() - 1 && 
				Math.abs(goalX - locX) < 0.05 &&
				Math.abs(goalY - locY) < 0.05){
				pathIndex++;
			}
			if (pathIndex >= goalPath.size()){
				done = true;
				setVelocities(0.0, 0.0);
			}

			goalX = goalPath.get(pathIndex).x;
			goalY = goalPath.get(pathIndex).y;
			goalTheta = Math.atan2(goalY - locY, goalX - locX);
			if (goalTheta < 0){
				goalTheta += 2 * Math.PI;
			}

			double error = goalTheta - locTheta;
			if (error > Math.PI){
				error -= 2 * Math.PI;
			} else if (error < - Math.PI){
				error += 2 * Math.PI;
			}
			if (Math.abs(error) < Math.PI / 8.0){
				setVelocities(0.3, anglePID.update(error));
			} else {
				setVelocities(0, anglePID.update(error));
			}
			//setVelocities(0.2, anglePID.update(error));
		}
	}

	private void setVelocities(double trans, double rot){
		org.ros.message.rss_msgs.MotionMsg msg = new org.ros.message.rss_msgs.MotionMsg();
		msg.translationalVelocity = trans;
		msg.rotationalVelocity = rot;
		motionPub.publish(msg);
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
		return new GraphName("rss/RobotNav");
	}
}
