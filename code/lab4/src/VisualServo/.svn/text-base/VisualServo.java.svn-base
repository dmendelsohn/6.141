package VisualServo;

import java.util.concurrent.ArrayBlockingQueue;

import org.ros.namespace.GraphName;
import org.ros.node.parameter.*;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.*;

/**
 *
 * @author previous TA's, prentice, vona
 *
 */
public class VisualServo implements NodeMain, Runnable {
	private boolean continuePublish = true;
	private int endCount = 0;

	private static final int width = 160;
	private static final int height = 120;

	/**
	 * <p>The blob tracker.</p>
	 **/
	private BlobTracking blobTrack = null;

	// Parameters for blockTrack velocity controlling (from Staff solution in Spring 2012)
	private double desired_fixation_distance = .5;
	private double translation_error_tolerance = .05;
	private double translation_velocity_gain = 2*.75;
	private double translation_velocity_max = .75;
	private double rotation_error_tolerance = 0.2;
	private double rotation_velocity_gain = 2*0.15;
	private double rotation_velocity_max = 0.15;

	private VisionGUI gui;
	private ArrayBlockingQueue<byte[]> visionImage = new ArrayBlockingQueue<byte[]>(
			1);

	protected boolean firstUpdate = true;

	public Subscriber<org.ros.message.sensor_msgs.Image> vidSub;
	public Subscriber<org.ros.message.rss_msgs.OdometryMsg> odoSub;
	public Publisher<MotionMsg> fixPub;  // Declare publisher
	// publishes centroidX, centroidY, width, height
	public Publisher<org.ros.message.rss_msgs.ResetMsg> imageStatsPub;

	/**
	 * <p>Create a new VisualServo object.</p>
	 */
	public VisualServo() {

		setInitialParams();

		//gui = new VisionGUI();
	}

	protected void setInitialParams() {

	}

	/**
	 * <p>Handle a CameraMessage. Perform blob tracking and
	 * servo robot towards target.</p>
	 *
	 * @param rawImage a received camera message
	 */
	public void handle(byte[] rawImage) {

		visionImage.offer(rawImage);
	}

	@Override
	public void run() {
		while (true) {
			Image src = null;
			try {
				src = new Image(visionImage.take(), width, height);
			} catch (InterruptedException e) {
				e.printStackTrace();
				continue;
			}

			Image dest = new Image(src);

			blobTrack.apply(src, dest);

			// update newly formed vision message
			//gui.setVisionImage(dest.toArray(), width, height);

			// publish velocity messages to move the robot towards the target
			if (continuePublish){
				MotionMsg msg = new MotionMsg();
				msg.translationalVelocity = blobTrack.translationVelocityCommand;
				msg.rotationalVelocity = blobTrack.rotationVelocityCommand;
				fixPub.publish(msg);
			}

			// stop publishing velocities if close enough
			System.out.println("CentroidX: " + blobTrack.centroidX);
			System.out.println("Fraction: " + (0.5 - (blobTrack.centroidX / blobTrack.width)));
			System.out.println("Continue Publish: " + continuePublish);
			if (Math.abs(0.5 - (blobTrack.centroidX / blobTrack.width)) < 0.05){
				endCount++;
			} else {
				endCount = 0;
			}

			if (endCount > 2){
				continuePublish = false;
			}

			// publish whether still publishing
			org.ros.message.rss_msgs.ResetMsg imageMsg = new org.ros.message.rss_msgs.ResetMsg();
			imageMsg.reset = continuePublish;
			imageStatsPub.publish(imageMsg);
		}
	}

	/**
	 * <p>
	 * Run the VisualServo process
	 * </p>
	 *
	 * @param node optional command-line argument containing hostname
	 */
	@Override
	public void onStart(Node node) {
		blobTrack = new BlobTracking(width, height);

		// Begin Student Code

		// set parameters on blobTrack
		blobTrack.desiredFixationDistance = desired_fixation_distance;
		blobTrack.translationErrorTolerance = translation_error_tolerance;
		blobTrack.translationVelocityGain = translation_velocity_gain;
		blobTrack.translationVelocityMax = translation_velocity_max;
		blobTrack.rotationErrorTolerance = rotation_error_tolerance;
		blobTrack.rotationVelocityGain = rotation_velocity_gain;
		blobTrack.rotationVelocityMax = rotation_velocity_max;

		System.err.println("  desired fixation distance: " + blobTrack.desiredFixationDistance);
		System.err.println("  translation error tolerance: " + blobTrack.translationErrorTolerance);
		System.err.println("  translation velocity gain: " + blobTrack.translationVelocityGain);
		System.err.println("  translation velocity max: " +  blobTrack.translationVelocityMax);
		System.err.println("  rotation error tolerance: " + blobTrack.rotationErrorTolerance);
		System.err.println("  rotation velocity gain: " + blobTrack.rotationVelocityGain);
		System.err.println("  rotation velocity max: " +  blobTrack.rotationVelocityMax); //(Solution)

		// initialize the ROS publication to command/Motors
		fixPub = node.newPublisher("command/Motors", "rss_msgs/MotionMsg");
		imageStatsPub = node.newPublisher("rss/ImageStats", "rss_msgs/ResetMsg");

		// End Student Code


		final boolean reverseRGB = node.newParameterTree().getBoolean("reverse_rgb", false);

		vidSub = node.newSubscriber("/rss/video", "sensor_msgs/Image");
		vidSub
		.addMessageListener(new MessageListener<org.ros.message.sensor_msgs.Image>() {
			@Override
			public void onNewMessage(
					org.ros.message.sensor_msgs.Image message) {
				if (continuePublish){
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
			}
		});

		odoSub = node.newSubscriber("/rss/odometry", "rss_msgs/OdometryMsg");
		odoSub
		.addMessageListener(new MessageListener<org.ros.message.rss_msgs.OdometryMsg>() {
			@Override
			public void onNewMessage(
					org.ros.message.rss_msgs.OdometryMsg message) {
				if (firstUpdate) {
					firstUpdate = false;
					//gui.resetWorldToView(message.x, message.y);
				}
				//gui.setRobotPose(message.x, message.y, message.theta);
			}
		});
		Thread runningStuff = new Thread(this);
		runningStuff.start();
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
		return new GraphName("rss/visualservo");
	}
}
