package Logging;

import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.*;
import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.parameter.*;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;

public class Logger implements NodeMain {
	public static final String LOGDIR = "log";
	public static final String LOGFILE_PREFIX = LOGDIR + "/log_";
	public static final String IMAGE_PREFIX = LOGDIR + "/images/image_";

	private Subscriber<org.ros.message.rss_msgs.OdometryMsg> odoSub;
	private Subscriber<org.ros.message.sensor_msgs.Image> vidSub;
	private String logFileName;

	public Logger() {
		logFileName = LOGFILE_PREFIX + System.currentTimeMillis() + ".txt";
		//TODO: instantiate Filo I/O stuff
	}

	@Override
	public void onStart(Node node) {
		odoSub = node.newSubscriber("/rss/Odometry", "rss_msgs/OdometryMsg");
		vidSub = node.newSubscriber("/rss/video", "sensor_msgs/Image");

		odoSub.addMessageListener(new MessageListener<org.ros.message.rss_msgs.OdometryMsg>() {
			@Override
			public void onNewMessage(org.ros.message.rss_msgs.OdometryMsg message) {
				handleOdometryMessage(message);
			}
		});

		bumpSub.addMessageListener(new MessageListener<org.ros.message.sensor_msgs.Image>() {
			@Override
			public void onNewMessage(org.ros.message.sensor_msgs.Image message) {
				handleImageMessage(message);
			}
		});
	}

	public void handleOdometryMessage(org.ros.message.rss_msgs.OdometryMsg message) {
		//TODO: append info to log file
	}

	public void handleImageMessage(org.ros.message.sensor_msgs.Image message) {
		//TODO: save image to file, and append info to log file
	}
}
