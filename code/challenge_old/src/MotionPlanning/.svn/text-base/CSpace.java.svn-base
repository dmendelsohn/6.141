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

public class CSpace {

	public static PolygonObstacle minkowskiSum(PolygonObstacle poly1, PolygonObstacle poly2){
		List<Point2D.Double> minkVertices = new LinkedList<Point2D.Double>();

		for (Point2D.Double pt1 : poly1.getVertices()){
			for (Point2D.Double pt2 : poly2.getVertices()){
				minkVertices.add(new Point2D.Double(pt1.x + pt2.x, pt1.y + pt2.y));
			}
		}

		return GeomUtils.convexHull(minkVertices);
	}

	public static PolygonObstacle reflect(PolygonObstacle poly, Point2D.Double center){
		PolygonObstacle imagePoly = new PolygonObstacle();

		for (Point2D.Double pt : poly.getVertices()){
			imagePoly.addVertex(2*center.x - pt.x, 2*center.y - pt.y);
		}

		imagePoly.close();

		return imagePoly;
	}

	public static PolygonObstacle CSpaceObstacle(PolygonObstacle obst, PolygonObstacle robot, Point2D.Double center){
		PolygonObstacle refRobot = reflect(robot, center);
		PolygonObstacle cObst = minkowskiSum(obst, robot);

		return cObst;
	}

	public static Rectangle2D.Double CSpaceWorldRect(Rectangle2D.Double worldRect, Point2D.Double center, double radius){
		return new Rectangle2D.Double(worldRect.getX() + radius, worldRect.getY() + radius, worldRect.getWidth() - 2 * radius,
			worldRect.getHeight() - 2 * radius);
	}

	public static List<PolygonObstacle> computeCSpace(PolygonMap map, PolygonObstacle robot, Point2D.Double center){
		List<PolygonObstacle> obstacles = map.getObstacles();
		List<PolygonObstacle> cObstacles = new LinkedList<PolygonObstacle>();

		for (PolygonObstacle obst : obstacles){
			cObstacles.add(CSpaceObstacle(obst, robot, center));
		}

		return cObstacles;
	}

}