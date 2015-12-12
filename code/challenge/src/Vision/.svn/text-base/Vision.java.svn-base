package Vision;

import java.util.concurrent.ArrayBlockingQueue;

import java.awt.Color;
import java.util.*;
import Challenge.*;
import java.awt.geom.*;
import MotionPlanning.*;
import MapFiles.*;

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
    - Remember to consider running connected components for each color if current approach does not work
*/




public class Vision implements NodeMain {
	// Constants
	private final double DAMP_VAL = 0.5;
	private final boolean SIMULATE = false;
	private final boolean LOCALIZE_REGULARLY = false;
	private final boolean LOCALIZE_ON_SIGHT = false;
	private final boolean LOCALIZE_ON_WAYPOINT = true;
	private final double RADIUS = 0.2;
	private final double LOCALIZATION_INTERVAL = 5000;
	private final int NUM_SIDES = 8;
	private double X_DISP = 0;
	private double Y_DISP = 0;
	private double THETA_DISP = 0;

	// Simulation Data
	private int locNum = 0;
	private List<List<FiducialBlob>> simData = new ArrayList<List<FiducialBlob>>();

	// State Variables
	private static final int width = 160;
	private static final int height = 120;
	private double odo_x = 0;
	private double odo_y = 0;
	private double odo_theta = 0;
	private double last_localization = 0;
	Point2D.Double robotStart = new Point2D.Double(0, 0);

	// Blob Tracker and Localization
	private BlobTracking blobTrack = null;
	//private Localization localization = null;
	private AngleLocalization localization = null;

	// GUI and Image Variables
	private VisionGUI gui;
	private ArrayBlockingQueue<byte[]> visionImage = new ArrayBlockingQueue<byte[]>(1);
	protected boolean firstUpdate = true;

	// Subscribers and Publishers
	public Subscriber<org.ros.message.sensor_msgs.Image> vidSub;
	//public Subscriber<org.ros.message.sensor_msgs.Image> depthSub;
	public Subscriber<org.ros.message.rss_msgs.OdometryMsg> odoSub;
	public Subscriber<org.ros.message.rss_msgs.ResetMsg> newGoalSub;
	public Publisher<org.ros.message.rss_msgs.OdometryMsg> localPub;
	public Publisher<org.ros.message.rss_msgs.ResetMsg> stopPub;
	//public Publisher<org.ros.message.rss_msgs.OdometryMsg> blockPub;

	// Polygon Map
	GrandChallengeMap gcm;
	String mapFileName;

	public Vision() {
		setInitialParams();
		gui = new VisionGUI();

		generateSimulationData();
	}

	@Override
	public void onStart(Node node) {
		ParameterTree paramTree = node.newParameterTree();
		mapFileName = paramTree.getString(node.resolveName("~/mapFileName"));

		last_localization = System.currentTimeMillis();

		try {
			gcm = GrandChallengeMap.parseFile(mapFileName);
		} catch (Exception e){
			e.printStackTrace();
		}

		robotStart = gcm.getRobotStart();

		Fiducial[] fiducials = gcm.getFiducials();

		Rectangle2D.Double cSpaceWorld = CSpace.CSpaceWorldRect(gcm.getWorldRect(), gcm.getRobotStart(), RADIUS);
		List<PolygonObstacle> cSpaceObstacles = CSpace.computeCSpace(gcm, getRobot(), gcm.getRobotStart());
		//localization = new Localization(cSpaceObstacles, cSpaceWorld, robotStart, fiducialPairs);
		localization = new AngleLocalization(cSpaceObstacles, cSpaceWorld, robotStart, fiducials);

		blobTrack = new BlobTracking(width, height);
		final boolean reverseRGB = node.newParameterTree().getBoolean("reverse_rgb", false);

		localPub = node.newPublisher("/rss/localization", "rss_msgs/OdometryMsg");
		//blockPub = node.newPublisher("/rss/block", "rss_msgs/OdometryMsg");
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

		/*depthSub = node.newSubscriber("/rss/depth", "sensor_msgs/Image");
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
		});*/

		odoSub = node.newSubscriber("/rss/odometry", "rss_msgs/OdometryMsg");
		odoSub.addMessageListener(new MessageListener<org.ros.message.rss_msgs.OdometryMsg>() {
			@Override
			public void onNewMessage(org.ros.message.rss_msgs.OdometryMsg message) {
				if (firstUpdate){
					X_DISP = robotStart.x - message.x;
					Y_DISP = robotStart.y - message.y;
					THETA_DISP = - message.theta;
					firstUpdate = false;
				}
				odo_x = message.x + X_DISP;
				odo_y = message.y + Y_DISP;
				odo_theta = message.theta + THETA_DISP;

				OdoPoint curr_point;
				//Point2D.Double curr_point;
				OdometryMsg msg = new OdometryMsg();
				if (LOCALIZE_REGULARLY || LOCALIZE_ON_WAYPOINT || LOCALIZE_ON_SIGHT){
					synchronized(localization){
						//curr_point = localization.encoderUpdate(odo_x, odo_y);
						curr_point = localization.encoderUpdate(odo_x, odo_y, odo_theta);
					}
					//msg.x = curr_point.x;
					//msg.y = curr_point.y;
					//msg.theta = curr_point.theta;

					msg.x = DAMP_VAL * curr_point.x + (1 - DAMP_VAL) * odo_x;
					msg.y = DAMP_VAL * curr_point.y + (1 - DAMP_VAL) * odo_y;
					msg.theta = DAMP_VAL * curr_point.theta + (1 - DAMP_VAL) * odo_theta;
				} else {
					msg.x = odo_x;
					msg.y = odo_y;
					msg.theta = odo_theta;
				}
				System.out.println("Vision: publishing localization = (" + msg.x + ", " + msg.y + ", " + msg.theta + ")");
				localPub.publish(msg);
			}
		});

		newGoalSub = node.newSubscriber("/rss/newgoal", "rss_msgs/ResetMsg");
		newGoalSub.addMessageListener(new MessageListener<org.ros.message.rss_msgs.ResetMsg>() {
			@Override
			public void onNewMessage(org.ros.message.rss_msgs.ResetMsg message) {
				if (!message.reset && LOCALIZE_ON_WAYPOINT){
					OdometryMsg msg = new OdometryMsg();
					ResetMsg stop_msg = new ResetMsg();
					stop_msg.reset = false;
					stopPub.publish(stop_msg);

					try {
						Thread.sleep(2000);
					} catch (Exception exc){
						exc.printStackTrace();
					}

					OdoPoint curr_point;
					//Point2D.Double curr_point;
					synchronized(localization){
						//curr_point = localization.localize(odo_x, odo_y, blobTrack.fiducials);
						System.out.println("Vision: beginning localization");
						if (SIMULATE){
							curr_point = localization.localize(odo_x, odo_y, odo_theta, simData.get(locNum));
							locNum++;
						} else {
							curr_point = localization.localize(odo_x, odo_y, odo_theta, blobTrack.fiducials);
						}
					}
					//msg.x = curr_point.x;
					//msg.y = curr_point.y;
					//msg.theta = curr_point.theta;

					msg.x = DAMP_VAL * curr_point.x + (1 - DAMP_VAL) * odo_x;
					msg.y = DAMP_VAL * curr_point.y + (1 - DAMP_VAL) * odo_y;
					msg.theta = DAMP_VAL * curr_point.theta + (1 - DAMP_VAL) * odo_theta;

					System.out.println("Vision: odometry = (" + odo_x + ", " + odo_y + ", " + odo_theta + ")");
					System.out.println("Vision: new localization = (" + msg.x + ", " + msg.y + ", " + msg.theta + ")");

					last_localization = System.currentTimeMillis();

					try {
						Thread.sleep(2000);
					} catch (Exception exc){
						exc.printStackTrace();
					}

					stop_msg = new ResetMsg();
					stop_msg.reset = true;
					stopPub.publish(stop_msg);
				}
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

			//Point2D.Double curr_point;
			OdoPoint curr_point;
			OdometryMsg msg = new OdometryMsg();
			if (LOCALIZE_REGULARLY && System.currentTimeMillis() - last_localization > LOCALIZATION_INTERVAL) {
				ResetMsg stop_msg = new ResetMsg();
				stop_msg.reset = false;
				stopPub.publish(stop_msg);

				synchronized(localization){
					//curr_point = localization.localize(odo_x, odo_y, blobTrack.fiducials);
					curr_point = localization.localize(odo_x, odo_y, odo_theta, blobTrack.fiducials);
				}
				msg.x = curr_point.x;
				msg.y = curr_point.y;
				msg.theta = curr_point.theta;

				last_localization = System.currentTimeMillis();

				try {
					Thread.sleep(500);
				} catch (Exception exc){
					exc.printStackTrace();
				}

				stop_msg = new ResetMsg();
				stop_msg.reset = true;
				stopPub.publish(stop_msg);
			} else if (LOCALIZE_ON_SIGHT && blobTrack.fiducials.size() > 0){
				ResetMsg stop_msg = new ResetMsg();
				stop_msg.reset = false;
				stopPub.publish(stop_msg);

				synchronized(localization){
					//curr_point = localization.localize(odo_x, odo_y, blobTrack.fiducials);
					curr_point = localization.localize(odo_x, odo_y, odo_theta, blobTrack.fiducials);
				}
				msg.x = curr_point.x;
				msg.y = curr_point.y;
				msg.theta = curr_point.theta;

				last_localization = System.currentTimeMillis();

				try {
					Thread.sleep(2000);
				} catch (Exception exc){
					exc.printStackTrace();
				}

				stop_msg = new ResetMsg();
				stop_msg.reset = true;
				stopPub.publish(stop_msg);
			} else if (LOCALIZE_ON_WAYPOINT){
				synchronized(localization){
					//curr_point = localization.encoderUpdate(odo_x, odo_y);
					curr_point = localization.encoderUpdate(odo_x, odo_y, odo_theta);
				}
				msg.x = curr_point.x;
				msg.y = curr_point.y;
				msg.theta = curr_point.theta;
			} else {
				msg.x = odo_x;
				msg.y = odo_y;
				msg.theta = odo_theta;
			}
			localPub.publish(msg);

			/*OdometryMsg blockMsg = new OdometryMsg();
			if (blobTrack.blocks.size() > 0){
				ImageBlob center = blobTrack.blocks.get(0);
				blockMsg.x = ((width / 2.0) - center.x) / width;
				blockMsg.y = (height - center.y) / height;
				blockMsg.theta = 0;
				//System.out.println("Vision: block found at msg coords = " + blockMsg.x + ", " + blockMsg.y);
				blockPub.publish(blockMsg);
			} else {
				blockMsg.x = 0;
				blockMsg.y = 0;
				blockMsg.theta = 1;
				blockPub.publish(blockMsg);
			}*/

			// Update newly formed vision message
			gui.setVisionImage(dest.toArray(), width, height);

			try {
				//Thread.sleep(50);
				Thread.sleep(1000);
			} catch (Exception exc){
				exc.printStackTrace();
			}

			/*ResetMsg stop_msg = new ResetMsg();
			stop_msg.reset = true;
			stopPub.publish(stop_msg);*/
		}
	}

	public PolygonObstacle getRobot(){
		Point2D.Double center = gcm.getRobotStart();
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

	private void generateSimulationData(){
		List<FiducialBlob> data = new ArrayList<FiducialBlob>();
		data.add(new FiducialBlob(new Color[]{Color.ORANGE, Color.RED}, 120, 0, 80));
		simData.add(data);

		for (int i = 0; i < 6; i++){
			simData.add(new ArrayList<FiducialBlob>());
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
		return new GraphName("rss/vision");
	}
}
