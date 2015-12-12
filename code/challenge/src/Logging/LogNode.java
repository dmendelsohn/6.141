package Logging;

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

public class LogNode implements NodeMain {
	public static final String LOGDIR = "/RSS-I-group/challenge/log";
	public static final String LOGFILE_PREFIX = LOGDIR + "/log_";
	public static final String IMAGE_PREFIX = LOGDIR + "/images/image_";

	private Subscriber<org.ros.message.rss_msgs.OdometryMsg> odoSub;
	private Subscriber<org.ros.message.sensor_msgs.Image> vidSub;
	private String logFileName;
	private Logger logger;
	private long startTime;

	private class MyFormatter extends java.util.logging.Formatter {
		public String format(LogRecord record) {
			return record.getMessage();
		}
	}

	public LogNode() {
		logFileName = LOGFILE_PREFIX + System.currentTimeMillis() + ".txt";
		logger = Logger.getLogger("ChallengeLog");
		FileHandler fh;

		try {
			fh = new FileHandler(logFileName);
			logger.addHandler(fh);
			//SimpleFormatter formatter = new SimpleFormatter();
			java.util.logging.Formatter formatter = new MyFormatter();
			fh.setFormatter(formatter);

			logger.info("My first log"); //DEBUGGING
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	@Override
	public void onStart(Node node) {
		odoSub = node.newSubscriber("/rss/Odometry", "rss_msgs/OdometryMsg");
		vidSub = node.newSubscriber("/rss/video", "sensor_msgs/Image");
		startTime = System.currentTimeMillis();

		odoSub.addMessageListener(new MessageListener<org.ros.message.rss_msgs.OdometryMsg>() {
			@Override
			public void onNewMessage(org.ros.message.rss_msgs.OdometryMsg message) {
				handleOdometryMessage(message);
			}
		});

		vidSub.addMessageListener(new MessageListener<org.ros.message.sensor_msgs.Image>() {
			@Override
			public void onNewMessage(org.ros.message.sensor_msgs.Image message) {
				handleImageMessage(message);
			}
		});
	}

	private void handleOdometryMessage(org.ros.message.rss_msgs.OdometryMsg message) {
		String content = "" + message.x + " " + message.y + " " + message.theta;
		String timestamp = "" + (System.currentTimeMillis() - startTime);
		logger.info(timestamp + " " + "/rss/odometry" + " " + content);
	}

	private void handleImageMessage(org.ros.message.sensor_msgs.Image message) {
		String timestamp = "" + (System.currentTimeMillis() - startTime);
		String filename = IMAGE_PREFIX + timestamp; //TODO: what's the format suffix?
		logger.info(timestamp + " " + "/rss/video" + " " + filename);
		//TODO: save image to file at filename
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
