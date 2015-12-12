package LocalNavigation;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;

import java.util.LinkedList;
import java.util.List;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.TimeUnit;

import utils.Point;

import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.SonarMsg;
import org.ros.message.rss_msgs.BumpMsg;
import org.ros.message.rss_msgs.MotionMsg;
import org.ros.message.lab5_msgs.GUIPointMsg;
import org.ros.message.lab5_msgs.GUILineMsg;
import org.ros.message.lab5_msgs.ColorMsg;
import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import LocalNavigation.SonarGUI;

public class LocalNavigation implements NodeMain {
	// Constants
	private final org.ros.message.lab5_msgs.ColorMsg RED_MSG;
	private final org.ros.message.lab5_msgs.ColorMsg BLUE_MSG;
	private final double MIN_OBSTACLE = 0.01;
	private final double MAX_OBSTACLE = 1.0;

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
	private List<Point> obstaclePoints;
	private double lineA = 0, lineB = 0, lineC = -1;
	private List<Line> totalLines;

	private double prevWallAngle = - 11;
	private double travelledAngle = 0;

	private Point goalPoint;
	private double desiredWallDist = 0.3; //in meters
	private PIDController anglePID; //for the true
	private PIDController trackingPID = new PIDController(1.0, 0, 0); //for wall tracking

	private double sonarDifferenceTolerence = 0.02; //the max distance the sonar differences can be before robot adjust. in meters

	// State
	private int state;
	private final int NO_STATE = -1;
	private final int STOP_ON_BUMP = 0;
	private final int ALIGN_ON_BUMP = 1;
	private final int ALIGNING = 2;
	private final int ALIGNED = 3;
	private final int ALIGNED_AND_ROTATING = 4;
	private final int ALIGNED_AND_ROTATED = 5;
	private final int BACKING_UP = 6;
	private final int FINDING_WALL = 7;
	private final int TRACKING_WALL = 8;
	private final int WALL_ENDED = 9;
	private final int CIRCLE_TO_BUMP = 10;
	private final int DONE = 11;

	// Sonar GUI
	private SonarGUI gui;

	// ROS Subscribers
	public Subscriber<org.ros.message.rss_msgs.SonarMsg> frontSonarSub;
	public Subscriber<org.ros.message.rss_msgs.SonarMsg> backSonarSub;
	public Subscriber<org.ros.message.rss_msgs.BumpMsg> bumpSub;
	public Subscriber<org.ros.message.rss_msgs.OdometryMsg> odoSub;

	// ROS Publishers
	public Publisher<org.ros.message.std_msgs.String> statePub;
	public Publisher<org.ros.message.rss_msgs.MotionMsg> fixPub;
	public Publisher<org.ros.message.lab5_msgs.GUIPointMsg> guiPtPub;
	public Publisher<org.ros.message.lab5_msgs.GUILineMsg> guiLinePub;
	public Publisher<org.ros.message.lab5_msgs.GUISegmentMsg> guiSegmentPub;

	private String logFilename = "/home/rss-student/RSS-I-group/lab5/out.txt";
	private FileWriter fileWriter;

	//Helper Functions
	private double capped(double value){
		return Math.min(2.0, value);
	}

	public LocalNavigation() {
		gui = new SonarGUI();
		state = ALIGN_ON_BUMP;

		RED_MSG = new org.ros.message.lab5_msgs.ColorMsg();
		RED_MSG.r = 255;
		RED_MSG.g = 0;
		RED_MSG.b = 0;

		BLUE_MSG = new org.ros.message.lab5_msgs.ColorMsg();
		BLUE_MSG.r = 0;
		BLUE_MSG.g = 0;
		BLUE_MSG.b = 255;

		frontSonarBuffer = new LinkedList<Double>();
		backSonarBuffer = new LinkedList<Double>();

		obstaclePoints = new LinkedList<Point>();
		totalLines = new LinkedList<Line>();
	}

	@Override
	public void onStart(Node node) {
		bumpSub = node.newSubscriber("/rss/BumpSensors", "rss_msgs/BumpMsg");
		frontSonarSub = node.newSubscriber("/rss/Sonars/Front", "rss_msgs/SonarMsg");
		backSonarSub = node.newSubscriber("/rss/Sonars/Back", "rss_msgs/SonarMsg");
		odoSub = node.newSubscriber("/rss/odometry", "rss_msgs/OdometryMsg");

		statePub = node.newPublisher("/rss/state", "std_msgs/String");
		fixPub = node.newPublisher("command/Motors", "rss_msgs/MotionMsg");
		guiPtPub = node.newPublisher("gui/Point", "lab5_msgs/GUIPointMsg");
		guiLinePub = node.newPublisher("gui/Line", "lab5_msgs/GUILineMsg");
		guiSegmentPub = node.newPublisher("gui/Segment", "lab5_msgs/GUISegmentMsg");

		frontSonarSub.addMessageListener(new MessageListener<org.ros.message.rss_msgs.SonarMsg>() {
			@Override
			public void onNewMessage(org.ros.message.rss_msgs.SonarMsg message) {
				updateSonars(message.range, true);
				handleState();
			}
		});

		backSonarSub.addMessageListener(new MessageListener<org.ros.message.rss_msgs.SonarMsg>() {
			@Override
			public void onNewMessage(org.ros.message.rss_msgs.SonarMsg message) {
				updateSonars(message.range, false);
				handleState();
			}
		});

		bumpSub.addMessageListener(new MessageListener<org.ros.message.rss_msgs.BumpMsg>() {
			@Override
			public void onNewMessage(org.ros.message.rss_msgs.BumpMsg message) {
				handleBumpSensors(message);
			}
		});

		odoSub.addMessageListener(new MessageListener<org.ros.message.rss_msgs.OdometryMsg>() {
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

				findObstacleLine();
				drawObstacleLine();
				handleState();
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

	private void updateWallReadings(){
		if (Math.min(frontSonar, backSonar) > 0 && Math.max(frontSonar, backSonar) < 0.4){
			double frontX = locX - frontSonar * Math.sin(locTheta);
			double frontY = locY + frontSonar * Math.cos(locTheta);

			obstaclePoints.add(new Point(frontX, frontY));

			double backX = locX - backSonar * Math.sin(locTheta);
			double backY = locY + backSonar * Math.cos(locTheta);

			obstaclePoints.add(new Point(backX, backY));
		}

		/*if (Math.max(frontSonar, backSonar) > MAX_OBSTACLE || Math.min(frontSonar, backSonar) < MIN_OBSTACLE){
			// || Math.abs(frontSonar - backSonar) > MAX_DIFFERENCE){
			if (obstacle){
				//drawObstacleSegment(obstaclePoints, new Line(lineA, lineB, lineC));
				totalObstaclePoints.add(new LinkedList<Point>(obstaclePoints));
				totalLines.add(new Line(lineA, lineB, lineC));
				obstaclePoints.clear();
			}
			obstacle = false;
		} else {
			if (frontSonar > 0 && backSonar > 0){
				double frontX = locX - frontSonar * Math.sin(locTheta);
				double frontY = locY + frontSonar * Math.cos(locTheta);

				obstaclePoints.add(new Point(frontX, frontY));

				double backX = locX - backSonar * Math.sin(locTheta);
				double backY = locY + backSonar * Math.cos(locTheta);

				obstaclePoints.add(new Point(backX, backY));
			}

			obstacle = true;
		}*/
	}

	private void findObstacleLine(){
		double X = 0, Y = 0, X2 = 0, Y2 = 0, Z = 0;
		for (Point pt : obstaclePoints){
			X += pt.x;
			Y += pt.y;
			X2 += pt.x * pt.x;
			Y2 += pt.y * pt.y;
			Z += pt.x * pt.y;
		}
		double D = X2 * Y2 - Z * Z;

		lineA = (X * Y2 - Y * Z) / D;
		lineB = (Y * X2 - X * Z) / D;
	}

	public void drawObstacleLine(){
		if (lineA != 0 || lineB != 0){
			org.ros.message.lab5_msgs.GUILineMsg msg = new org.ros.message.lab5_msgs.GUILineMsg();
			msg.lineA = lineA;
			msg.lineB = lineB;
			msg.lineC = lineC;
			msg.color = BLUE_MSG;

			guiLinePub.publish(msg);
		}
	}

	public void drawObstacleSegment(List<Point> prevObstacle, Line line){
		Point start = prevObstacle.get(0);
		Point end = prevObstacle.get(prevObstacle.size() - 1);

		// draw line
		double startK = (line.a * start.x + line.b * start.y + line.c) / (line.a * line.a + line.b * line.b);
		double startX = start.x - startK * line.a;
		double startY = start.y - startK * line.b;

		double endK = (line.a * end.x + line.b * end.y + line.c) / (line.a * line.a + line.b * line.b);
		double endX = end.x - endK * line.a;
		double endY = end.y - endK * line.b;

		org.ros.message.lab5_msgs.GUISegmentMsg msg = new org.ros.message.lab5_msgs.GUISegmentMsg();
		msg.startX = startX;
		msg.startY = startY;
		msg.endX = endX;
		msg.endY = endY;
		msg.color = BLUE_MSG;

		guiSegmentPub.publish(msg);
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

	private void handleState(){
		publishState();
		switch(state) {
			case STOP_ON_BUMP:
				if (bumpLeft || bumpRight){
					setVelocities(0, 0);
				} else {
					setVelocities(0.3, 0);
				}
				break;

			case ALIGN_ON_BUMP:
				if (bumpLeft || bumpRight){
					state = ALIGNING;
				} else {
					setVelocities(0.2, 0);
				}
				break;

			case ALIGNING:
				if (bumpLeft && bumpRight){
					state = ALIGNED;
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
				break;

			case ALIGNED:
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
				break;

			case ALIGNED_AND_ROTATING:
				System.out.println("CURR " + locTheta);
				System.out.println("GOAL " + goalPoint.theta);
				double error = goalPoint.theta - locTheta;

				if (Math.abs(error) > 0.05){
					setVelocities(0, anglePID.update(error));
				} else {
					setVelocities(0, 0);
					state = ALIGNED_AND_ROTATED;
				}
				break;

			case ALIGNED_AND_ROTATED:
				state = BACKING_UP;
				break;

			case BACKING_UP:
				//If we see an obstacle and are in BACKING_UP State, start backing up
				if(Math.max(backSonar, frontSonar) < 0.6){
					setVelocities(-.3, -trackingPID.update(capped(backSonar) - capped(frontSonar)));
				} else {
					setVelocities(0,0);
					state = FINDING_WALL;
				}
				break;

			case FINDING_WALL:
				updateWallReadings();
				if (Math.max(backSonar, frontSonar) > 0.6){
					setVelocities(.3, 0.0);
				} else {
					setVelocities(0.0,0.0);
					state = TRACKING_WALL;
				}
				break;

			case TRACKING_WALL:
				System.out.println("Back, Front: " + backSonar + ", " + frontSonar);
				updateWallReadings();
				if (Math.max(backSonar, frontSonar) < 0.6){
					updateWallReadings();
					setVelocities(.3, -trackingPID.update(capped(backSonar) - capped(frontSonar)));
					double translationError = desiredWallDist - 0.5*(frontSonar + backSonar);
					double rotationError = (backSonar - frontSonar);
					long timeElapsedMillis = System.currentTimeMillis(); //TODO improve
				} else {
					System.out.println("drawing segment");
					findObstacleLine();
					drawObstacleSegment(obstaclePoints, new Line(lineA, lineB, lineC));
					obstaclePoints.clear();
					setVelocities(0,0);
					state = WALL_ENDED;
				}
				break;

			case WALL_ENDED:
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
				} else {
					state = CIRCLE_TO_BUMP;
				}
				break;

			case CIRCLE_TO_BUMP:
				if (bumpLeft || bumpRight){
					state = ALIGNING;
				} else {
					setVelocities(0.3, 0.1);
				}
				break;

			case DONE:
				setVelocities(0, 0);
				break;

			default:
				System.out.println("Improper state of robot");
		}
		publishState();
	}

	private void publishState(){
		String msg;
		if (state == STOP_ON_BUMP){
			msg = "STOP_ON_BUMP";
		} else if (state == ALIGN_ON_BUMP){
			msg = "ALIGN_ON_BUMP";
		} else if (state == ALIGNING){
			msg = "ALIGNING";
		} else if (state == ALIGNED){
			msg = "ALIGNED";
		} else if (state == ALIGNED_AND_ROTATED){
			msg = "ALIGNED_AND_ROTATED";
		} else if (state == BACKING_UP){
			msg = "BACKING_UP";
		} else if (state == FINDING_WALL){
			msg = "FINDING_WALL";
		} else if (state == TRACKING_WALL){
			msg = "TRACKING_WALL";
		} else if (state == WALL_ENDED){
			msg = "WALL_ENDED";
		} else if (state == CIRCLE_TO_BUMP){
			msg = "CIRCLE_TO_BUMP";
		} else if (state == DONE){
			msg = "DONE";
		} else {
			msg = "State not yet added to publishState";
		}
		System.out.println(msg);
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

	private class PIDController {
		double gainP, gainI, gainD, prevTime;
		double prevError = 0;
		double integral = 0;

		PIDController(double gainP, double gainI, double gainD){
			this.gainP = gainP;
			this.gainI = gainI;
			this.gainD = gainD;

			prevTime = System.currentTimeMillis();
		}

		public double update(double error){
			double deriv = (error - prevError) / (System.currentTimeMillis() - prevTime);
			integral += (System.currentTimeMillis() - prevTime) * error;
			double out = gainP * error + gainI * integral + gainD * deriv;

			prevTime = System.currentTimeMillis();
			prevError = error;

			return out;
		}
	}

	private class Line {
		public double a, b, c;

		Line(double a, double b, double c){
			this.a = a;
			this.b = b;
			this.c = c;
		}
	}
}
