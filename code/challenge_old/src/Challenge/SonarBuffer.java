/*
 * This is a helper class for processing sonar input
 * */


package Challenge;

import java.util.*;

import org.ros.message.MessageListener;
import org.ros.message.rss_msgs.*;
import org.ros.message.challenge_msgs.*;
import org.ros.node.topic.Subscriber;

public class SonarBuffer {
	public static final int DEFAULT_SIZE = 5;

	private List<Double> frontSonarBuffer;
	private List<Double> backSonarBuffer;
	private int maxBufferSize;

	public SonarBuffer() {
		this(DEFAULT_SIZE);
	}

	public SonarBuffer(int maxBufferSize) {
		this.maxBufferSize = maxBufferSize;
		this.frontSonarBuffer = new ArrayList<Double>();
		this.backSonarBuffer = new ArrayList<Double>();
	}

	public void updateSonars(double val, boolean isFront) {
		if (isFront){
			updateBuffer(frontSonarBuffer, val);
		} else {
			updateBuffer(backSonarBuffer, val);
		}
	}

	public double getFrontSonar() {
		return getBufferAverage(frontSonarBuffer);
	}

	public double getBackSonar() {
		return getBufferAverage(backSonarBuffer);
	}

	private void updateBuffer(List<Double> buffer, double newValue) {
		buffer.add(newValue);
		if (buffer.size() > maxBufferSize) {
			buffer.remove(0);
		}
	}

	private double getBufferAverage(List<Double> buffer) {
		if (buffer == null || buffer.size() == 0) {
			return 0.0;
		}
		double sum = 0;
		for (double val : buffer) {
			sum += val;
		}
		return sum / buffer.size();
	}
}
