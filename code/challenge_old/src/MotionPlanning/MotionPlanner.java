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




/* NOTES FOR IMPLEMENTATION:
    - Have not implemented retrieving list of block locations from the map
    - Update localization to work with failures
*/




public class MotionPlanner implements NodeMain {
	// Localization State
	private double local_x = 0;
	private double local_y = 0;
	private double local_theta = 0;

	// Constants
	private final org.ros.message.lab5_msgs.ColorMsg RED_MSG;
	private final org.ros.message.lab5_msgs.ColorMsg BLUE_MSG;
	private final int NUM_SIDES = 8;
	private final double RADIUS = 0.2;
	private final int GRID_WIDTH = 60;
	private final int GRID_HEIGHT = 60;
	private double SQUARE_WIDTH;
	private double SQUARE_HEIGHT;

	// Configuration Space
	List<PolygonObstacle> cSpaceObstacles;
	Rectangle2D.Double cSpaceWorld;
	Point2D.Double robotGoal;
	List<Point2D.Double> goalPath;
	List<Point2D.Double> mappedBlocks;

	// ROS Subscribers
	public Subscriber<org.ros.message.rss_msgs.OdometryMsg> localSub;

	// ROS Publishers
	public Publisher<org.ros.message.lab6_msgs.GUIRectMsg> rectPub;
	public Publisher<org.ros.message.lab6_msgs.GUIPolyMsg> polyPub;
	public Publisher<org.ros.message.lab5_msgs.GUISegmentMsg> segPub;
	public Publisher<org.ros.message.lab6_msgs.GUIPolyMsg> pathPub;

	// Map GUI
	MapGUI gui;

	// Polygon Map
	PolygonMap polymap;
	String mapFileName;

	public MotionPlanner(){
		gui = new MapGUI();

		RED_MSG = new org.ros.message.lab5_msgs.ColorMsg();
		RED_MSG.r = 255;
		RED_MSG.g = 0;
		RED_MSG.b = 0;

		BLUE_MSG = new org.ros.message.lab5_msgs.ColorMsg();
		BLUE_MSG.r = 0;
		BLUE_MSG.g = 0;
		BLUE_MSG.b = 255;

		goalPath = new LinkedList<Point2D.Double>();
	}

	/**
	 * Entry hook for ROS when called as stand-alone node
	 */
	@Override
	public void onStart(Node node) {
		ParameterTree paramTree = node.newParameterTree();
   		mapFileName = paramTree.getString(node.resolveName("~/mapFileName"));

		try {
			polymap = new PolygonMap(mapFileName);
		} catch (Exception e){
			e.printStackTrace();
		}

		rectPub = node.newPublisher("gui/Rect", "lab6_msgs/GUIRectMsg");
		polyPub = node.newPublisher("gui/Poly", "lab6_msgs/GUIPolyMsg");
		segPub = node.newPublisher("gui/Segment", "lab5_msgs/GUISegmentMsg");
		pathPub = node.newPublisher("command/path", "lab6_msgs/GUIPolyMsg");

		try {
			Thread.sleep(500);
		} catch (Exception e){
			e.printStackTrace();
		}

		displayMap();
		updateCSpace();
		displayCSpace();

		localSub = node.newSubscriber("/rss/localization", "rss_msgs/OdometryMsg");
		localSub.addMessageListener(new MessageListener<org.ros.message.rss_msgs.OdometryMsg>() {
			@Override
			public void onNewMessage(org.ros.message.rss_msgs.OdometryMsg message) {
				local_x = message.x;
				local_y = message.y;
				local_theta = message.theta;
			}
		});
	}

	public void searchForPath(){
		System.out.println("Starting search");
		findRobotGoal();

		List<Point2D.Double> queue = new ArrayList<Point2D.Double>();
		List<Point2D.Double> visited = new ArrayList<Point2D.Double>();
		Map<Point2D.Double, Point2D.Double> parents = new HashMap<Point2D.Double, Point2D.Double>();

		Point2D.Double start = new Point2D.Double(SQUARE_WIDTH * Math.floor(local_x / SQUARE_WIDTH),
			SQUARE_HEIGHT * Math.floor(local_y / SQUARE_HEIGHT));

		queue.add(start);
		parents.put(start, null);
		visited.add(start);

		Point2D.Double currPoint = new Point2D.Double(1000, 1000);

		System.out.println("Starting search");
		int count = 0;
		while (queue.size() > 0){
			currPoint = queue.get(0);
			queue.remove(0);
			//visited.add(currPoint);

			count++;
			//System.out.println("Count, Point: " + count + ", " + currPoint.x + ", " + currPoint.y);

			if (currPoint.x <= robotGoal.x &&
				robotGoal.x <= currPoint.x + SQUARE_WIDTH &&
				currPoint.y <= robotGoal.y &&
				robotGoal.y <= currPoint.y + SQUARE_HEIGHT){
				break;
			}

			for (Point2D.Double child : getChildren(currPoint, visited)){
				//System.out.println("Child: " + child.x + ", " + child.y);
				parents.put(child, currPoint);
				queue.add(child);
				visited.add(child);
			}
		}

		//goalPath = new LinkedList<Point2D.Double>();
		goalPath.clear();
		goalPath.add(robotGoal);
		currPoint = parents.get(currPoint);

		while (currPoint != null){
			goalPath.add(0, new Point2D.Double(currPoint.x + 0.5 * SQUARE_WIDTH,
				currPoint.y + 0.5 * SQUARE_HEIGHT));

			currPoint = parents.get(currPoint);
		}

		goalPath.remove(0);
		goalPath.add(0, new Point2D.Double(local_x, local_y));

		GUIPolyMsg msg = new GUIPolyMsg();
		int size = goalPath.size();
		float[] xCoords = new float[size];
		float[] yCoords = new float[size];
		for (int i = 0; i < size; i++) {
			Point2D.Double point = goalPath.get(i);
			xCoords[i] = (float) (point.x);
			yCoords[i] = (float) (point.y);
		}
		msg.x = xCoords;
		msg.y = yCoords;
		msg.numVertices = size;
		pathPub.publish(msg);
	}

	public List<Point2D.Double> getChildren(Point2D.Double pt, List<Point2D.Double> visited){
		List<Point2D.Double> children = new LinkedList<Point2D.Double>();
		Rectangle2D.Double currRect;
		double x, y;
		boolean intersects = false;
		boolean seen = false;
		// Reversed to prefer right
		for (int i = 1; i >= -1; i--){
			for (int j = -1; j <= 1; j++){
				if (i != 0 || j != 0){
					x = pt.x + i * SQUARE_WIDTH;
					y = pt.y + j * SQUARE_HEIGHT;

					if (x > cSpaceWorld.getX() - 0.5 * SQUARE_WIDTH &&
						x < cSpaceWorld.getX() + cSpaceWorld.getWidth() - 0.5 * SQUARE_WIDTH &&
						y > cSpaceWorld.getY() - 0.5 * SQUARE_HEIGHT &&
						y < cSpaceWorld.getY() + cSpaceWorld.getHeight() - 0.5 * SQUARE_HEIGHT){
						currRect = new Rectangle2D.Double(x, y, SQUARE_WIDTH, SQUARE_HEIGHT);
						intersects = false;
						seen = false;
						for (PolygonObstacle obst : cSpaceObstacles){
							if (obst.intersects(currRect)){
								intersects = true;
							}
						}
						for (Point2D.Double vPoint : visited){
							//System.out.println("Children check: " + vPoint.x + ", " + vPoint.y + ", " + x + ", " + y);
							if (Math.abs(vPoint.x - x) < 0.25 * SQUARE_WIDTH &&
								Math.abs(vPoint.y - y) < 0.25 * SQUARE_HEIGHT){
								seen = true;
							}
						}
						if (!intersects && !seen){
							children.add(new Point2D.Double(x, y));
						}
					}
				}
			}
		}

		return children;
	}

	public void findRobotGoal(){
		if (mappedBlocks.size() > 0){
			Collections.sort(mappedBlocks, new Comparator<Point2D.Double>(){
				public int compare( Point2D.Double o1, Point2D.Double o2 ) {
					double distance1 = Math.sqrt((o1.x - local_x) * (o1.x - local_x) + (o1.y - local_y) * (o1.y - local_y));
					double distance2 = Math.sqrt((o2.x - local_x) * (o2.x - local_x) + (o2.y - local_y) * (o2.y - local_y));
					if (distance1 == distance2){
						return 0;
					} else if (distance1 < distance2){
						return 1;
					} else {
						return -1;
					}
				}
			});
			robotGoal = mappedBlocks.get(0);
			mappedBlocks.remove(0);
		} else {
			Rectangle2D.Double currRect;
			boolean intersects;
			double x, y;
			while (true){
				x = cSpaceWorld.getX() + cSpaceWorld.getWidth() * Math.random();
				y = cSpaceWorld.getY() + cSpaceWorld.getHeight() * Math.random();

				currRect = new Rectangle2D.Double(x, y, SQUARE_WIDTH, SQUARE_HEIGHT);
				intersects = false;
				for (PolygonObstacle obst : cSpaceObstacles){
					if (obst.intersects(currRect)){
						intersects = true;
					}
				}
				if (!intersects){
					break;
				}
			}
			robotGoal = new Point2D.Double(x, y);
		}
	}

	public void updateCSpace(){
		PolygonObstacle robot = getRobot();
		Point2D.Double center = polymap.getRobotStart();

		local_x = polymap.getRobotStart().x;
		local_y = polymap.getRobotStart().y;

		// Fix this to actually implement getting list of blocks
		mappedBlocks = polymap.getBlocks();

		cSpaceWorld = CSpace.CSpaceWorldRect(polymap.getWorldRect(), center, RADIUS);
		cSpaceObstacles = CSpace.computeCSpace(polymap, robot, center);

		SQUARE_WIDTH = cSpaceWorld.getWidth() / GRID_WIDTH;
		SQUARE_HEIGHT = cSpaceWorld.getHeight() / GRID_HEIGHT;
	}

	public void displayPath(){
		for (int i = 0 ; i < goalPath.size() - 1; i++){
			org.ros.message.lab5_msgs.GUISegmentMsg msg = new org.ros.message.lab5_msgs.GUISegmentMsg();
			msg.startX = (float) goalPath.get(i).x;
			msg.startY = (float) goalPath.get(i).y;

			msg.endX = (float) goalPath.get(i + 1).x;
			msg.endY = (float) goalPath.get(i + 1).y;

			msg.color = RED_MSG;

			segPub.publish(msg);
		}
	}

	public void displayMap(){
		Rectangle2D.Double world = polymap.getWorldRect();
		displayRectangle(world, 0, true);

		List<PolygonObstacle> obstacles = polymap.getObstacles();

		//System.out.println("DISPLAYING MAP");

		for (PolygonObstacle obst : obstacles){
			displayObstacle(obst, 1, true);
		}
	}

	public void displayCSpace(){
		displayRectangle(cSpaceWorld, 0, false);

		for (PolygonObstacle obst : cSpaceObstacles){
			displayObstacle(obst, 0, false);
		}
	}

	public PolygonObstacle getRobot(){
		Point2D.Double center = polymap.getRobotStart();
		PolygonObstacle robot = new PolygonObstacle();

		double x, y, angle;
		for (int i = 0; i < NUM_SIDES; i++){
			angle = i * 2.0 * Math.PI / NUM_SIDES;
			x = RADIUS * Math.cos(angle);
			y = RADIUS * Math.sin(angle);
			robot.addVertex(x ,y);
		}

		robot.close();

		return robot;
	}

	private void displayRectangle(Rectangle2D.Double rect, int filled, boolean isRed){
		org.ros.message.lab6_msgs.GUIRectMsg msg = new org.ros.message.lab6_msgs.GUIRectMsg();
		if (isRed){
			msg.c = RED_MSG;
		} else {
			msg.c = BLUE_MSG;
		}
		msg.x = (float) rect.x;
		msg.y = (float) rect.y;
		msg.width = (float) rect.width;
		msg.height = (float) rect.height;
		msg.filled = filled;

		rectPub.publish(msg);
	}

	private void displayObstacle(PolygonObstacle obstacle, int filled, boolean isRed){
		org.ros.message.lab6_msgs.GUIPolyMsg msg = new org.ros.message.lab6_msgs.GUIPolyMsg();
		if (isRed){
			msg.c = RED_MSG;
		} else {
			msg.c = BLUE_MSG;
		}
		int size = obstacle.getVertices().size();
		msg.numVertices = size;
		msg.x = new float[size];
		msg.y = new float[size];
		msg.closed = 1;
		msg.filled = filled;

		for (int i = 0; i < size; i++){
			msg.x[i] = (float) obstacle.getVertices().get(i).x;
			msg.y[i] = (float) obstacle.getVertices().get(i).y;
		}

		polyPub.publish(msg);
	}

	public void testConvexHull(){
		List<Point2D.Double> points = new LinkedList<Point2D.Double>();
		for (int i = -3; i < -1; i++){
			for (int j = 1; j < 3; j++){
				points.add(new Point2D.Double(i, j));
			}
		}
		points.add(new Point2D.Double(-4, 0));

		PolygonObstacle hull = GeomUtils.convexHull(points);

		displayObstacle(hull, 0, true);
	}

	public static void fillPolyMsg(GUIPolyMsg msg, PolygonObstacle obstacle, java.awt.Color c, boolean filled, boolean closed) {

	}

	public static void fillRectMsg(GUIRectMsg msg, java.awt.geom.Rectangle2D.Double r, java.awt.Color c, boolean filled){

	}

	public void handle(BumpMsg arg0) {

	}

	public void handle(OdometryMsg arg0) {

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
		return new GraphName("rss/globalnavigation");
	}

}
