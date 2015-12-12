package VisualServo;

import java.awt.Color;
import java.util.List;
import java.util.ArrayList;
import java.util.Map;
import java.util.Collections;
import java.util.HashMap;
import java.util.Comparator;
import java.util.Map.Entry;

/**
 * BlobTracking performs image processing and tracking for the VisualServo
 * module.  BlobTracking filters raw pixels from an image and classifies blobs,
 * generating a higher-level vision image.
 *
 * @author previous TA's, prentice
 */
public class BlobTracking {
	protected int stepCounter = 0;
	protected double lastStepTime = 0.0;

	public int width;
	public int height;

	// Variables used for velocity controller that are available to calling
	// process.  Visual results are valid only if targetDetected==true; motor
	// velocities should do something sane in this case.
	public boolean targetDetected = false; // set in blobPresent()
	public double centroidX = 0.0; // set in blobPresent()
	public double centroidY = 0.0; // set in blobPresent()
	public double targetArea = 0.0; // set in blobPresent()
	public double targetRange = 0.0; // set in blobFix()
	public double targetBearing = 0.0; // set in blobFix()

	// System variables for velocity control in Old Staff Solution (Spring 2012)
	// These variables are set in VisualServo.java
	/*public double desiredFixationDistance;
	public double translationErrorTolerance;
	public double rotationErrorTolerance;
	public double translationVelocityGain;
	public double translationVelocityMax;
	public double rotationVelocityGain;
	public double rotationVelocityMax;*/

	public double desiredFixationDistance = .75;
	public double translationErrorTolerance = .05;
	public double translationVelocityGain = 2*.75;
	public double translationVelocityMax = .75;
	public double rotationErrorTolerance = 0.2;
	//public double rotationVelocityGain = 2*0.15;
	//public double rotationVelocityGain = 0.005;
	public double rotationVelocityGain = 0.5;
	public double rotationVelocityMax = 0.15;

	// Commands to be published to uorc motor driver
	public double translationVelocityCommand = 0.0;
	public double rotationVelocityCommand = 0.0;

	public double cameraHeight = 37.0; 
	public double cameraOffset = 10.0;
	public double focalPlaneDistance = 107.0; //(Solution)


	/**
	 * <p>Create a BlobTracking object</p>
	 *
	 * @param width image width
	 * @param height image height
	 */
	public BlobTracking(int width, int height) {

		this.width = width;
		this.height = height;

	}

	/**
	 * <p>Computes frame rate of vision processing</p>
	 */
	private void stepTiming() {
		double currTime = System.currentTimeMillis();
		stepCounter++;
		// if it's been a second, compute frames-per-second
		if (currTime - lastStepTime > 1000.0) {
			//double fps = (double) stepCounter * 1000.0
			// / (currTime - lastStepTime);
			//System.err.println("FPS: " + fps);
			stepCounter = 0;
			lastStepTime = currTime;
		}
	}

	/**  //(Old Staff Solution)
	 * 	<p>Compute the translational velocity command using a //(Old Staff Solution)
	 *  P-controller on current translation error.</p>
	 *
	 * 	@return translational velocity command
	 */
	protected void computeTranslationVelocityCommand() {
		double translationError = targetRange/100 - desiredFixationDistance;
		if (Math.abs(translationError) < translationErrorTolerance) {
			translationVelocityCommand = 0.0;
		} else {
			translationVelocityCommand = 
					Math.max(-translationVelocityMax, 
							Math.min(translationVelocityMax, translationError * translationVelocityGain));
		}
	}

	/** //(Old Staff Solution)
	 * <p>Compute the rotational velocity command using a P-controller
	 * on the current rotation error, {@link #targetBearing}.<\p>
	 *
	 * @return rotation velocity command
	 */
	protected void computeRotationVelocityCommand() {
		double rotationError = targetBearing;
		if (Math.abs(rotationError) < rotationErrorTolerance) {
			rotationVelocityCommand = 0.0;
		} else {
			rotationVelocityCommand = 
					Math.max(-rotationVelocityMax,
							Math.min(rotationVelocityMax, -rotationError * rotationVelocityGain)); 	 	 	 										
		}
	}

	/**
	 * <p>Segment out a blob from the src image (if a good candidate exists).</p>
	 *
	 * <p><code>dest</code> is a packed RGB image for a java image drawing
	 * routine. If it's not null, the blob is highlighted.</p>
	 *
	 * @param src the source RGB image, not packed
	 * @param dest the destination RGB image, packed, may be null
	 */
	public void apply(Image src, Image dest) {

		stepTiming(); // monitors the frame rate

		// Print central pixel values
		//printAvgCentralPixels(src, 0.2, true);

		// Display RGB histogram
		//dest = Histogram.getHistogram(src, dest, false);

		// Display HSB histogram
		//dest = Histogram.getHistogram(src, dest, true);

		// Blob Pixel
		blobPixel(src, dest, 0);

		// Blob Present
		blobPresent(src, 0);

		if (targetDetected){
			drawCentroid(src, dest, 0);
			blobFix();
			computeTranslationVelocityCommand();
			computeRotationVelocityCommand();
		}
		else{
			rotationVelocityCommand = 0.0;
			translationVelocityCommand = 0.0;
		}

		// System.out.println("///Blob Information:///");
		// System.out.println("Area="+targetArea);
		// System.out.println("Centroid= (" + String.valueOf(centroidX) + ", " + String.valueOf(centroidY) + ")");

		// double radius = Math.sqrt(targetArea/Math.PI);
		// System.out.println("Radius="+radius);

		// double CONVERSION_FACTOR = 0.0063368;
		// double BALL_RADIUS = 4.1;

		// double distance = BALL_RADIUS / Math.sin(CONVERSION_FACTOR * radius);
		// double floor_distance = Math.sqrt(distance * distance - 37 * 37) + 7;

		// System.out.println("Distance="+targetRange);
		// System.out.println("Bearing="+targetBearing);
		// System.out.println("Trans Vel="+translationVelocityCommand);
		// System.out.println("Rotate Vel="+rotationVelocityCommand);

	}

	/**
	 * <p> Prints average pixel values from a central region of the image
	 */
	public void printAvgCentralPixels(Image src, double fraction, boolean hsb){
		int startX = (int) ((1 - fraction) * src.getWidth() / 2);
		int endX = (int) ((1 + fraction) * src.getWidth() / 2);
		int startY = (int) ((1 - fraction) * src.getHeight() / 2);
		int endY = (int) ((1 + fraction) * src.getHeight() / 2);

		// Either RGB or HSB in that order
		double a = 0;
		double b = 0;
		double c = 0;

		int counter = 0;

		float[] hsb_vals;
		for (int i = startX; i <= endX; i++){
			for (int j = startY; j <= endY; j++){
				if (hsb){
					hsb_vals = Color.RGBtoHSB(Image.pixelByteToInt(src.getPixelRed(i, j)),
							Image.pixelByteToInt(src.getPixelGreen(i, j)),
							Image.pixelByteToInt(src.getPixelBlue(i, j)), null);
					
					a += hsb_vals[0];
					b += hsb_vals[1];
					c += hsb_vals[2];
					
					counter++;
				} else {
					a += Image.pixelByteToInt(src.getPixelRed(i, j));
					b += Image.pixelByteToInt(src.getPixelGreen(i, j));
					c += Image.pixelByteToInt(src.getPixelBlue(i, j));

					counter++;
				}
			}
		}

		a /= counter;
		b /= counter;
		c /= counter;

		if (hsb){
			System.out.println("********************");
			System.out.println("Average Hue: " + String.valueOf(a));
			System.out.println("Average Saturation: " + String.valueOf(b));
			System.out.println("Average Value: " + String.valueOf(c));
		} else {
			System.out.println("********************");
			System.out.println("Average Red: " + String.valueOf(a));
			System.out.println("Average Green: " + String.valueOf(b));
			System.out.println("Average Blue: " + String.valueOf(c));
		}
	}

	/**
	 * <p> Classifies each pixel in the image as a target blob (ball) pixel or not. Takes in
	 *  a color option specifying detection setting (0 = red, 1 = green, 2 = blue, 3 = all)
	 */
	public void blobPixel(Image src, Image dest, int color){
		int width = src.getWidth();
		int height = src.getHeight();

		byte[] destpix = new byte[3 * width * height];
		GaussianBlur.apply(src.toArray(), destpix, width, height);
		src = new Image(destpix, width, height);

		int red, green, blue;
		int[] write_values;

		for (int i = 0; i < width; i++){
			for (int j = 0; j < height; j++){
				red = Image.pixelByteToInt(src.getPixelRed(i, j));
				green = Image.pixelByteToInt(src.getPixelGreen(i, j));
				blue = Image.pixelByteToInt(src.getPixelBlue(i, j));

				write_values = classifyPixel(red, green, blue, false, color);
				dest.setPixel(i, j, (byte) write_values[0], (byte) write_values[1], (byte) write_values[2]);
			}
		}
	}

	/**
	 *  Classifies a pixel in terms of RGB values as a ball or background. Takes in
	 *  a color option specifying detection setting (0 = red, 1 = green, 2 = blue, 3 = all)
	 */
	private int[] classifyPixel(int red, int green, int blue, boolean set_zero, int color){
		int[] output = new int[3];

		float[] hsb = Color.RGBtoHSB(red, green, blue, null);
		float hue = hsb[0];
		float sat = hsb[1];

		//if ((color == 0 || color == 3) && (hue < 0.05 || hue > 0.95) && sat > 0.1){
		if ((color == 0 || color == 3) && (hue < 0.1 || hue > 0.9) && sat > 0.3){
			output[0] = 255;
			output[1] = 0;
			output[2] = 0;
		} else if ((color == 1 || color == 3) && hue < 0.38 && hue > 0.28 && sat > 0.5){
			output[0] = 0;
			output[1] = 255;
			output[2] = 0;
		} else if ((color == 2 || color == 3) && hue > 0.61 && hue < 0.71 && sat > 0.5){
			output[0] = 0;
			output[1] = 0;
			output[2] = 255;
		} else {
			if (set_zero){
				output[0] = 0;
				output[1] = 0;
				output[2] = 0;
			} else {
				output[0] = (int) ((red + green + blue)/ 3.0);
				output[1] = (int) ((red + green + blue)/ 3.0);
				output[2] = (int) ((red + green + blue)/ 3.0);
			}
		}

		return output;
	}

	/**
	 * <p> Identifies whether or not a ball is present and updates class fields including centroid. Takes
	 *  in a color option specifying detection setting (0 = red, 1 = green, 2 = blue, 3 = all)
	 */
	public void blobPresent(Image src, int color) {
		double THRESHOLD = 0.0005;

		int width = src.getWidth();
		int height = src.getHeight();

		// Apply Gaussian blur
		byte[] destpix = new byte[3 * width * height];
		GaussianBlur.apply(src.toArray(), destpix, width, height);
		src = new Image(destpix, width, height);

		// Find connected components
		int red, green, blue;

		int[] class_src = new int[width * height];
		int[] write_values;

		for (int i = 0; i < width; i++){
			for (int j = 0; j < height; j++){
				red = Image.pixelByteToInt(src.getPixelRed(i, j));
				green = Image.pixelByteToInt(src.getPixelGreen(i, j));
				blue = Image.pixelByteToInt(src.getPixelBlue(i, j));

				write_values = classifyPixel(red, green, blue, true, color);
				if (write_values[0] == 0 && write_values[1] == 0 && write_values[2] == 0){
					class_src[width * j + i] = 0;
				} else {
					class_src[width * j + i] = 1;
				}
			}
		}

		ConnectedComponents labeller = new ConnectedComponents();

		int[] dest = new int[width * height];
		dest = labeller.doLabel(class_src, dest, width, height);

		// Find second largest (largest non-background) connected component
		Map<Integer, Integer> color_sizes = new HashMap<Integer, Integer>();

		int curr_count = 0;
		int x, y;
		for (int i = 0; i < width * height; i++){
			x = i % width;
			y = (int) (i / width);

			red = Image.pixelByteToInt(src.getPixelRed(x, y));
			green = Image.pixelByteToInt(src.getPixelGreen(x, y));
			blue = Image.pixelByteToInt(src.getPixelBlue(x, y));

			if (color_sizes.containsKey(dest[i]) && (classifyPixel(red, green, blue, false, color)[0] == 255)){
				curr_count = color_sizes.get(dest[i]);
				color_sizes.put(dest[i], curr_count + 1);
			} else if (classifyPixel(red, green, blue, false, color)[0] == 255) {
				color_sizes.put(dest[i], 1);
			}
		}

		List<Entry<Integer, Integer>> pairs = new ArrayList<Entry<Integer, Integer>>(color_sizes.entrySet());

		Collections.sort(pairs, new Comparator<Map.Entry<Integer, Integer>>(){
			public int compare( Map.Entry<Integer, Integer> o1, Map.Entry<Integer, Integer> o2 ) {
				return (o2.getValue()).compareTo( o1.getValue() );
			}
		});

		// Report if a ball is found or not
		if (pairs.size() >= 1 && (pairs.get(0).getValue() > THRESHOLD * width * height)){
			int largestComp = pairs.get(0).getKey();
			int largestCompSize = pairs.get(0).getValue();

			System.out.println("BALL FOUND");
			targetDetected = true;
			targetArea = 0;

			centroidX = 0.0;
			centroidY = 0.0;

			for (int i = 0; i < width * height; i++){
				if (dest[i] == largestComp){
					x = i % width;
					y = (int) (i / width);

					centroidX += x;
					centroidY += y;

					targetArea++;
				}
			}

			centroidX /= targetArea;
			centroidY /= targetArea;

		} else {
			System.out.println("BALL LOST");
			targetDetected = false;
			centroidX = 0.0;
			centroidY = 0.0;
			targetArea = 0;
		}
	}

	/**
	 * <p> Draw in current centroid
	 */
	public void drawCentroid(Image src, Image dest, int color){
		int x, y;
		for (int i = -4; i < 5; i++){
			for (int j = -4; j < 5; j++){
				x = (int) (centroidX + i);
				y = (int) (centroidY + j);
				if (x >= 0 && y >= 0 && x < src.getWidth() && y < src.getHeight()){
					if (color == 0){
						dest.setPixel(x, y, (byte) 0, (byte) 255, (byte) 255);
					} else if (color == 1){
						dest.setPixel(x, y, (byte) 255, (byte) 0, (byte) 255);
					} else if (color == 2){
						dest.setPixel(x, y, (byte) 255, (byte) 255, (byte) 0);
					} else {
						dest.setPixel(x, y, (byte) 255, (byte) 255, (byte) 255);
					}
				}
			}
		}
	}

	/**
	 * Helper function to take distanceToRobot -> distanceToCamera
	 **/
	private double distanceToFloor(double distanceToCamera){
		return Math.sqrt(distanceToCamera*distanceToCamera - cameraHeight*cameraHeight)-10;
	}

	/**
	 * Helper function to take distanceToRobot -> distanceToCamera
	 **/
	private double distanceToCamera(double distanceToRobot){
		return Math.sqrt((distanceToRobot+cameraOffset)*(distanceToRobot+cameraOffset) + cameraHeight*cameraHeight);
	}

	/**
	 * Compute range and bearing.
	 *
	 * Using pinhole camera assumption (Horn, Ch2).
	 * For a point in the world (x,y,z) projected on to image plane IP as  
	 * (xp,yp,fp), where fp is the focal length, by simple geometry we know that: 
	 *
	 * xp/fp = x/z, yp/fp=y/z, or fp=xp*z/x //(Solution)
	 *  
	 *   
	 * Experimentally, we determine that: 
	 * 0.4m = radius of 13.55 pixels
	 * 0.8m = radius of 8.175 pixels
	 * 1.2m = radius of 5.3 pixels
	 * 
	 * We can estimate fp ~= Rpixel*Depth/Rblob, where Rblob ~= .13m , so fp //(Solution)
	 * ~= 107 pixels if we use the 0.5m estimate.</p> //(Solution)
	 * 
	 * Now given a area, we can estimate the depth as: Depth = fp*Rblob/Rpixel.
	 * We can also estimate the bearing using fp as: 
	 * bearing = atan2(centroidX,fp) 
	 * 
	 **/ 
	private void blobFix() {
		double CONVERSION_FACTOR = 0.0063368;
		double BALL_RADIUS = 4.1;
		double PIXEL_RADIUS = Math.sqrt(targetArea/Math.PI);

		targetRange = BALL_RADIUS / Math.sin(CONVERSION_FACTOR * PIXEL_RADIUS); //(Solution)
		double deltaX = centroidX - width / 2.0; //(Solution)
		targetBearing = Math.atan2(deltaX, focalPlaneDistance); //(Solution)
		// targetRange = //(Solution)
		// 	focalPlaneDistance * targetRadius / Math.sqrt(targetArea / Math.PI); //(Solution)
		// targetBearing = Math.atan2(deltaX, focalPlaneDistance); //(Solution)
	}

}
