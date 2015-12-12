package Vision;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import org.openni.*;

import org.ros.namespace.GraphName;
import org.ros.node.parameter.*;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.message.sensor_msgs.Image;

public class XtionVideo implements NodeMain, VideoStream.NewFrameListener {
  // ROS Publisher
  public Publisher<org.ros.message.sensor_msgs.Image> vidPub;
  public Publisher<org.ros.message.sensor_msgs.Image> depthPub;
  Device device;
  VideoStream rgbStream;
  VideoStream depthStream;
  VideoFrameRef lastRGBFrame;
  VideoFrameRef lastDepthFrame;

  /**
   * Entry hook for ROS when called as stand-alone node
   */
  @Override
  public void onStart(Node node) {
    vidPub = node.newPublisher("/rss/video", "sensor_msgs/Image");
    depthPub = node.newPublisher("/rss/depth", "sensor_msgs/Image");
    OpenNI.initialize();
    device = Device.open();
    System.out.println(device.getDeviceInfo());

    // initiate frame streams
    rgbStream = VideoStream.create(device, SensorType.COLOR);
    depthStream = VideoStream.create(device, SensorType.DEPTH);
    rgbStream.addNewFrameListener(this);
    depthStream.addNewFrameListener(this);
    rgbStream.start();
    depthStream.start();
  }

  @Override
  public synchronized void onFrameReady(VideoStream stream) {
    Image msg = new Image();
    if (stream.getSensorType() == SensorType.COLOR) {
      if (lastRGBFrame != null) {
        lastRGBFrame.release();
        lastRGBFrame = null;
      }
      lastRGBFrame = stream.readFrame();

      System.out.println(lastRGBFrame.getWidth());
      System.out.println(lastRGBFrame.getHeight());

      ByteBuffer frameData = lastRGBFrame.getData().order(ByteOrder.BIG_ENDIAN);
      byte[] data = new byte[frameData.capacity()];
      frameData.get(data);
      msg.data = data;
      msg.width = lastRGBFrame.getWidth();
      msg.height = lastRGBFrame.getHeight();
      msg.is_bigendian = 1;
      msg.step = lastRGBFrame.getStrideInBytes();
      msg.encoding = "rgb8";
      vidPub.publish(msg);

    } else if (stream.getSensorType() == SensorType.DEPTH) {
      if (lastDepthFrame != null) {
        lastDepthFrame.release();
        lastDepthFrame = null;
      }
      lastDepthFrame = stream.readFrame();

      System.out.println(lastDepthFrame.getWidth());
      System.out.println(lastDepthFrame.getHeight());

      ByteBuffer frameData = lastDepthFrame.getData().order(ByteOrder.BIG_ENDIAN);
      byte[] data = new byte[frameData.capacity()];
      frameData.get(data);
      msg.data = data;
      msg.width = lastDepthFrame.getWidth();
      msg.height = lastDepthFrame.getHeight();
      msg.is_bigendian = 1;
      msg.step = lastDepthFrame.getStrideInBytes();
      //msg.encoding = "16UC1";
      msg.encoding = "mono8";
      depthPub.publish(msg);

    } else {
      System.out.println("ERROR: unsupported sensor type in XtionVideo.java");
    }
  }


  @Override
  public void onShutdown(Node node) {
    if (node != null) {
      node.shutdown();
    }
    if (lastRGBFrame != null) {
      lastRGBFrame.release();
    }
    if (lastDepthFrame != null) {
      lastDepthFrame.release();
    }
    rgbStream.stop();
    rgbStream.destroy();
    depthStream.stop();
    depthStream.destroy();
    device.close();
  }

  @Override
  public void onShutdownComplete(Node node) {

  }

  @Override
  public GraphName getDefaultNodeName() {
    return new GraphName("rss/xtionvideo");
  }
}
