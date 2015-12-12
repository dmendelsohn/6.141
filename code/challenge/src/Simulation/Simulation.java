package Simulation;

import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.*;
import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.parameter.*;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import java.util.logging.*;
import java.util.*;
import java.io.*;

public class Simulation implements NodeMain {
	public static final String LOG_FILE_NAME = "log/log.txt";
	
	private Publisher<org.ros.message.rss_msgs.OdometryMsg> odoPub;
	private Publisher<org.ros.message.sensor_msgs.Image> vidPub;

	private BufferedReader reader;
	private long startTime;

	@Override
	public void onStart(Node node) {
		odoPub = node.newPublisher("/rss/odometry", "rss_msgs/OdometryMsg");
		vidPub = node.newPublisher("/rss/video", "sensor_msgs/Image");

		try {
			reader = new BufferedReader(new FileReader(LOG_FILE_NAME));
			String currentLine;
			startTime = System.currentTimeMillis();
			while ((currentLine = reader.readLine()) != null) {
				String[] strings = currentLine.split(" ");
				long timestamp = Long.valueOf(strings[0]);
				while (System.currentTimeMillis() - startTime < timestamp) {
					//block
				}
				String topic = strings[1];
				if (topic.equals("/rss/odometry")) {
					org.ros.message.rss_msgs.OdometryMsg message = new org.ros.message.rss_msgs.OdometryMsg();
					message.x = Float.valueOf(strings[2]);
					message.y = Float.valueOf(strings[3]);
					message.theta = Float.valueOf(strings[4]);
					odoPub.publish(message);
				} else if (topic.equals("/rss/video")) {
					String imageFile = strings[2];
					//TODO: load image from file and publish
				}
			}
		} catch (IOException e) {
			e.printStackTrace();
		}	
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
		return new GraphName("rss/motionplanner");
	}

}
