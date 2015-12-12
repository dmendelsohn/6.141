package Vision;

import java.util.concurrent.ArrayBlockingQueue;

import java.util.*;
import Challenge.*;
import java.awt.geom.*;
import MotionPlanning.*;

import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.*;
import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.parameter.*;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;





/* NOTES FOR IMPLEMENTATION:
    - Have not added in fiducial pairs
    - May want to implement better system for if localizing or not
    - Potentially replace Xtion with old camera
    - Potentially add a second camera for fiducials and keep one for servoing
    - Remember to cut off search for fiducials at a horizon line
    - Remember to update localization to account for errors
*/




public class Vision implements NodeMain {
	// Constants
	private final boolean LOCALIZE = true;
	private final double RADIUS = 0.2;
	private final double LOCALIZATION_INTERVAL = 5000;
	private final int NUM_SIDES = 8;

	// State Variables
	private static final int width = 320;
	private static final int height = 240;
	private double odo_x = 0;
	private double odo_y = 0;
	private double odo_theta = 0;
	private double last_localization = 0;

	// Blob Tracker and Localization
	private BlobTracking blobTrack = null;
	private Localization localization = null;

	// GUI and Image Variables
	private VisionGUI gui;
	private ArrayBlockingQueue<byte[]> visionImage = new ArrayBlockingQueue<byte[]>(1);
	protected boolean firstUpdate = true;

	// Subscribers and Publishers
	public Subscriber<org.ros.message.sensor_msgs.Image> vidSub;
	public Subscriber<org.ros.message.sensor_msgs.Image> depthSub;
	public Subscriber<org.ros.message.rss_msgs.OdometryMsg> odoSub;
	public Publisher<org.ros.message.rss_msgs.OdometryMsg> localPub;
	public Publisher<org.ros.message.rss_msgs.ResetMsg> stopPub;

	// Polygon Map
	PolygonMap polymap;
	String mapFileName;

	public Vision() {
		setInitialParams();
		gui = new VisionGUI();
	}

	@Override
	public void onStart(Node node) {
		ParameterTree paramTree = node.newParameterTree();
		mapFileName = paramTree.getString(node.resolveName("~/mapFileName"));

		last_localization = System.currentTimeMillis();

		try {
			polymap = new PolygonMap(mapFileName);
		} catch (Exception e){
			e.printStackTrace();
		}

		Point2D.Double robotStart = polymap.getRobotStart();

		Rectangle2D.Double cSpaceWorld = CSpace.CSpaceWorldRect(polymap.getWorldRect(), polymap.getRobotStart(), RADIUS);
		List<PolygonObstacle> cSpaceObstacles = CSpace.computeCSpace(polymap, getRobot(), polymap.getRobotStart());

		Map<int[], Point2D.Double> fiducialPairs = new HashMap<int[], Point2D.Double>();

		// Finish adding to fiducial pairs here from map

		localization = new Localization(cSpaceObstacles, cSpaceWorld, robotStart, fiducialPairs);

		blobTrack = new BlobTracking(width, height);
		final boolean reverseRGB = node.newParameterTree().getBoolean("reverse_rgb", false);

		localPub = node.newPublisher("/rss/localization", "rss_msgs/OdometryMsg");
		stopPub = node.newPublisher("/rss/stop", "rss_msgs/ResetMsg");

		vidSub = node.newSubscriber("/rss/video", "sensor_msgs/Image");
		vidSub.addMessageListener(new MessageListener<org.ros.message.sensor_msgs.Image>() {
			@Override
			public void onNewMessage(
				org.ros.message.sensor_msgs.Image message) {
				byte[] rgbData;
				if (reverseRGB) {
					rgbData = Image.RGB2BGR(message.data,
						(int) message.width, (int) message.height);
				}
				else {
					rgbData = message.data;
				}
				assert ((int) message.width == width);
				assert ((int) message.height == height);
				handle(rgbData);
			}
		});

		depthSub = node.newSubscriber("/rss/depth", "sensor_msgs/Image");
		depthSub.addMessageListener(new MessageListener<org.ros.message.sensor_msgs.Image>() {
			@Override
			public void onNewMessage(
				org.ros.message.sensor_msgs.Image message) {
				byte[] rgbData;
				if (reverseRGB) {
					rgbData = Image.RGB2BGR(message.data,
						(int) message.width, (int) message.height);
				}
				else {
					rgbData = message.data;
				}
				assert ((int) message.width == width);
				assert ((int) message.height == height);
				handle(rgbData);
			}
		});

		odoSub = node.newSubscriber("/rss/odometry", "rss_msgs/OdometryMsg");
		odoSub.addMessageListener(new MessageListener<org.ros.message.rss_msgs.OdometryMsg>() {
			@Override
			public void onNewMessage(org.ros.message.rss_msgs.OdometryMsg message) {
				odo_x = message.x;
				odo_y = message.y;
				odo_theta = message.theta;

				Point2D.Double curr_point;
				OdometryMsg msg = new OdometryMsg();
				if (LOCALIZE){
					synchronized(localization){
						curr_point = localization.encoderUpdate(odo_x, odo_y);
					}
					msg.x = curr_point.x;
					msg.y = curr_point.y;
				} else {
					msg.x = odo_x;
					msg.y = odo_y;
				}
				msg.theta = odo_theta;
				localPub.publish(msg);
			}
		});

		Thread visionThread = new Thread(new Runnable(){
			@Override
			public void run(){
				handleImage();
				try{
					Thread.sleep(10);
				} catch (Exception exc){
					exc.printStackTrace();
				}
			}
		});
		visionThread.start();
	}

	protected void setInitialParams() {

	}

	public void handle(byte[] rawImage) {
		visionImage.offer(rawImage);
	}

	public void handleImage() {
		while (true) {
			Image src = null;
			try {
				src = new Image(visionImage.take(), width, height);
			} catch (InterruptedException e) {
				e.printStackTrace();
				continue;
			}
			
			// Apply Blob Track to Image
			Image dest = new Image(src);
			blobTrack.apply(src, dest);

			OdometryMsg msg = new OdometryMsg();

			if (LOCALIZE && System.currentTimeMillis() - last_localization > LOCALIZATION_INTERVAL){
				ResetMsg stop_msg = new ResetMsg();
				stop_msg.reset = false;
				stopPub.publish(stop_msg);

				Point2D.Double curr_point;
				synchronized(localization){
					curr_point = localization.localize(odo_x, odo_y, blobTrack.fiducials);
				}
				msg.x = curr_point.x;
				msg.y = curr_point.y;

				last_localization = System.currentTimeMillis();
			} else {
				msg.x = odo_x;
				msg.y = odo_y;
			}
			msg.theta = odo_theta;
			localPub.publish(msg);

			// Update newly formed vision message
			gui.setVisionImage(dest.toArray(), width, height);

			try {
				Thread.sleep(1000);
			} catch (Exception exc){
				exc.printStackTrace();
			}

			ResetMsg stop_msg = new ResetMsg();
			stop_msg.reset = true;
			stopPub.publish(stop_msg);
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
		return new GraphName("rss/vision");
	}
}
