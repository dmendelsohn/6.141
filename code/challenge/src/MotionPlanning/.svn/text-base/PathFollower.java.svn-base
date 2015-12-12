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
	public Publisher<org.ros.message.rss_msgs.ResetMsg> newGoalPub;

	// State Variables
	private double locX = 0;
	private double locY = 0;
	private double locTheta = 0;
	private double goalTheta = 0;
	private List<Point2D.Double> goalPath;
	private int pathIndex = 0;
	private double goalX = 0;
	private double goalY = 0;
	private PIDController anglePID;
	private PIDController forwardPID;
	private int goalNumber = 1;

	public PathFollower() {
		goalPath = new LinkedList<Point2D.Double>();
		anglePID = new PIDController(0.15, 0, 0, 2, 2, 2);
		forwardPID = new PIDController(1, 0, 0, 0.2, 0.2, 0.2);
	}

	/**
	 * Entry hook for ROS when called as stand-alone node
	 */
	@Override
	public void onStart(Node node) {
		motionPub = node.newPublisher("/command/PathFollowing", "rss_msgs/MotionMsg");
		newGoalPub = node.newPublisher("/rss/newgoal", "rss_msgs/ResetMsg");

		localSub = node.newSubscriber("/rss/localization", "rss_msgs/OdometryMsg");
		pathSub = node.newSubscriber("/command/path", "lab6_msgs/GUIPolyMsg");

		pathSub.addMessageListener(new MessageListener<org.ros.message.lab6_msgs.GUIPolyMsg>() {
			@Override
			public void onNewMessage(org.ros.message.lab6_msgs.GUIPolyMsg message) {
				if (message.numVertices > 0){
					System.out.println("PathFollower: obtained new path of length " + message.numVertices);
				}

				/*for (int i = 0; i < message.numVertices; i++){
					Point2D.Double point = new Point2D.Double(message.x[i], message.y[i]);
					goalPath.add(point);
				}*/

				for (int i = 1; i < message.numVertices; i++){
					Point2D.Double point = new Point2D.Double(message.x[i], message.y[i]);
					goalPath.add(point);
				}
			}
		});

		localSub.addMessageListener(new MessageListener<org.ros.message.rss_msgs.OdometryMsg>() {
			@Override
			public void onNewMessage(org.ros.message.rss_msgs.OdometryMsg message) {
				locX = message.x;
				locY = message.y;
				locTheta = message.theta;
				
				navigateRobot();
			}
		});
	}

	public void navigateRobot(){
		//System.out.println("PathFollower: path index = " + pathIndex);
		//System.out.println("PathFollower: goal " + goalNumber + " path size = " + goalPath.size());
		//System.out.println("PathFollower: goal " + goalNumber + " (x, y, theta) = " + goalX + ", " + goalY + ", " + goalTheta);
		if (goalPath.size() > 0){
			locTheta = locTheta - ((int) (locTheta / (2 * Math.PI)));
			if (locTheta > Math.PI){
				locTheta -= 2 * Math.PI;
			}
			//System.out.println("PathFollower: location theta " + locTheta);

			goalX = goalPath.get(pathIndex).x;
			goalY = goalPath.get(pathIndex).y;
			goalTheta = Math.atan2(goalY - locY, goalX - locX);
			/*if (goalTheta < 0){
				goalTheta += 2 * Math.PI;
			}*/

			double error = goalTheta - locTheta;
			if (error > Math.PI){
				error -= 2 * Math.PI;
			} else if (error < - Math.PI){
				error += 2 * Math.PI;
			}
			//System.out.println("PathFollower: (goalTheta, locTheta, error) = " + goalTheta + ", " + locTheta + ", " + error);
			if (Math.abs(error) < Math.PI / 24.0){
				double distance = Math.sqrt((goalX - locX) * (goalX - locX) + (goalY - locY) * (goalY - locY));
				setVelocities(forwardPID.update(distance), anglePID.update(error));
				//setVelocities(0.2, anglePID.update(error));
			} else {
				setVelocities(0.0, anglePID.update(error));
			}
			//setVelocities(0.2, anglePID.update(error));

			/*if (pathIndex == goalPath.size() - 1 && 
				Math.abs(goalX - locX) < 0.1 &&
				Math.abs(goalY - locY) < 0.1){
				pathIndex++;
			}*/
			/*if (pathIndex >= goalPath.size()){
				System.out.println("PathFollower: finished path and requesting new path");
				goalPath.clear();
				pathIndex = 0;
				setVelocities(0.0, 0.0);
				ResetMsg msg = new ResetMsg();
				msg.reset = false;
				newGoalPub.publish(msg);
				goalNumber++;
				return;
			}*/
			if (pathIndex == goalPath.size() - 1 && 
				Math.abs(goalX - locX) < 0.05 &&
				Math.abs(goalY - locY) < 0.05){

				System.out.println("PathFollower: finished path and requesting new path");
				goalPath.clear();
				pathIndex = 0;
				setVelocities(0.0, 0.0);
				//setVelocities(0.15, 0.0);
				ResetMsg msg = new ResetMsg();
				msg.reset = false;
				newGoalPub.publish(msg);
				goalNumber++;
				return;
			}

			if (pathIndex < goalPath.size() - 1 &&
				Math.abs(goalX - locX) < 0.05 &&
				Math.abs(goalY - locY) < 0.05){
				pathIndex++;
			}
		} else {
			setVelocities(0, 0);
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
		return new GraphName("rss/pathfollower");
	}
}
