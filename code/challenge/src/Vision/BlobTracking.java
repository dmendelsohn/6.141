package Vision;

import java.awt.Color;
import java.util.*;
import java.util.Map.Entry;

import Challenge.*;
import Vision.*;

public class BlobTracking {
	// State Variables
	protected int stepCounter = 0;
	protected double lastStepTime = 0.0;

	public int width;
	public int height;

	public List<FiducialBlob> fiducials;
	public List<ImageBlob> blocks;

	// Measured Constants
	private final double MIN_FRACTIONAL_AREA = 0.001;
	private final double SUFFICIENTLY_CLOSE = 0.15;
	private final List<int[]> COLOR_MAP;
	private final List<Color> COLOR_CONVERSION;
	private final List<int[]> FIDUCIAL_COLOR_MAP;
	private final List<Color> FIDUCIAL_COLOR_CONVERSION;

	public BlobTracking(int width, int height) {
		this.width = width;
		this.height = height;

		fiducials = new LinkedList<FiducialBlob>();
		blocks = new LinkedList<ImageBlob>();

		COLOR_MAP = new LinkedList<int[]>();
		COLOR_MAP.add(new int[]{0, 0, 0});
		COLOR_MAP.add(new int[]{255, 0, 0});
		COLOR_MAP.add(new int[]{0, 255, 0});
		COLOR_MAP.add(new int[]{0, 0, 255});
		COLOR_MAP.add(new int[]{255, 255, 0});

		COLOR_CONVERSION = new LinkedList<Color>();
		COLOR_CONVERSION.add(Color.BLACK);
		COLOR_CONVERSION.add(Color.RED);
		COLOR_CONVERSION.add(Color.GREEN);
		COLOR_CONVERSION.add(Color.BLUE);
		COLOR_CONVERSION.add(Color.YELLOW);

		FIDUCIAL_COLOR_MAP = new LinkedList<int[]>();
		FIDUCIAL_COLOR_MAP.add(new int[]{0, 0, 0});
		FIDUCIAL_COLOR_MAP.add(new int[]{255, 0, 0});
		FIDUCIAL_COLOR_MAP.add(new int[]{0, 255, 0});
		FIDUCIAL_COLOR_MAP.add(new int[]{0, 0, 255});
		FIDUCIAL_COLOR_MAP.add(new int[]{255, 255, 0});
		FIDUCIAL_COLOR_MAP.add(new int[]{255, 127, 0});

		FIDUCIAL_COLOR_CONVERSION = new LinkedList<Color>();
		FIDUCIAL_COLOR_CONVERSION.add(Color.BLACK);
		FIDUCIAL_COLOR_CONVERSION.add(Color.RED);
		FIDUCIAL_COLOR_CONVERSION.add(Color.GREEN);
		FIDUCIAL_COLOR_CONVERSION.add(Color.BLUE);
		FIDUCIAL_COLOR_CONVERSION.add(Color.YELLOW);
		FIDUCIAL_COLOR_CONVERSION.add(Color.ORANGE);
	}

	private void stepTiming() {
		double currTime = System.currentTimeMillis();
		stepCounter++;
		// if it's been a second, compute frames-per-second
		if (currTime - lastStepTime > 1000.0) {
			double fps = (double) stepCounter * 1000.0 / (currTime - lastStepTime);
			//System.err.println("FPS: " + fps);
			stepCounter = 0;
			lastStepTime = currTime;
		}
	}

	public void apply(Image src, Image dest) {
		stepTiming(); // monitors the frame rate
		updateFiducials(src);
		System.out.println("BlobTracking: number of fiducials = " + fiducials.size());
		/*for (FiducialBlob fid : fiducials){
			System.out.println("BlobTracking: distance to fiducial " + ImageBlob.computeDistance(fid.size, 0.05));
			System.out.println("BlobTracking: bearing to fiducial " + ImageBlob.computeBearing(fid.x));
		}*/

		// Display RGB histogram
		//dest = Histogram.getHistogram(src, dest, false);

		// Display HSB histogram
		//dest = Histogram.getHistogram(src, dest, true);

		// Display Classification
		//drawBlobs(src, dest);
		drawFiducials(src, dest);

		// Draw Centroid of largest component
		//drawCentroid(src, dest, 0);
		drawFiducialCentroids(src, dest);
	}

	private int classifyPixel(int red, int green, int blue, boolean set_zero){
		float[] hsb = Color.RGBtoHSB(red, green, blue, null);
		float hue = hsb[0];
		float sat = hsb[1];

		if ((hue < 0.05 || hue > 0.95) && sat > 0.5){
			return 1; // red
		} else if (hue < 0.45 && hue > 0.35 && sat > 0.5){
			return 2; // green
		} else if (hue > 0.58 && hue < 0.68 && sat > 0.35){
			return 3; // blue
		} else if (hue > 0.1 && hue < 0.2 && sat > 0.5){
			return 4; // yellow
		} else {
			if (set_zero){
				return 0;
			} else {
				return -(red + green + blue)/ 3;
			}
		}
	}

	private int fiducialClassifyPixel(int red, int green, int blue, boolean set_zero){
		float[] hsb = Color.RGBtoHSB(red, green, blue, null);
		float hue = hsb[0];
		float sat = hsb[1];

		if (hue > 0 && hue < 0.05 && sat > 0.3) {
			return 1; // Red
		} else if (hue > 0.4 && hue < 0.49 && sat > 0.3) {
			return 2; // Green
		} else if (hue > 0.5 && hue < 0.59 && sat > 0.3) {
			return 3; // Blue
		} else if (hue > 0.12 && hue < 0.18 && sat > 0.4) {
			return 4; // Yellow
		} else if (hue > 0.05 && hue < 0.08 && sat > 0.3) {
			return 5; // Orange
		} else {
			if (set_zero){
				return 0;
			} else {
				return -(red + green + blue)/ 3;
			}
		}
	}

	public void updateFiducials(Image src){
		List<ImageBlob> indFiducials = new ArrayList<ImageBlob>();

		// Apply Gaussian blur
		byte[] destpix = new byte[3 * width * height];
		GaussianBlur.apply(src.toArray(), destpix, width, height);
		src = new Image(destpix, width, height);

		int red, green, blue;
		int[] class_src = new int[width * height];
		int[] write_values;
		int x, y;
		ImageBlob curr_blob;
		Map<Integer, ImageBlob> image_blobs = new HashMap<Integer, ImageBlob>();
		int[] dest = new int[width * height];

		for (int checkColor = 1; checkColor < 6; checkColor++){
			class_src = new int[width * height];

			// Find connected components
			for (int i = 0; i < width; i++){
				for (int j = 0; j < height; j++){
					red = Image.pixelByteToInt(src.getPixelRed(i, j));
					green = Image.pixelByteToInt(src.getPixelGreen(i, j));
					blue = Image.pixelByteToInt(src.getPixelBlue(i, j));

					if (fiducialClassifyPixel(red, green, blue, true) == checkColor){
						class_src[width * j + i] = 1;
					} else {
						class_src[width * j + i] = 0;
					}
				}
			}

			ConnectedComponents labeller = new ConnectedComponents();

			dest = new int[width * height];
			dest = labeller.doLabel(class_src, dest, width, height);

			// Find all connected components exceeding a minimum label
			image_blobs = new HashMap<Integer, ImageBlob>();

			for (int i = 0; i < width * height; i++){
				x = i % width;
				y = (int) (i / width);

				red = Image.pixelByteToInt(src.getPixelRed(x, y));
				green = Image.pixelByteToInt(src.getPixelGreen(x, y));
				blue = Image.pixelByteToInt(src.getPixelBlue(x, y));

				if (fiducialClassifyPixel(red, green, blue, true) == checkColor){
					if (image_blobs.containsKey(dest[i])){
						image_blobs.get(dest[i]).update(x, y);
					} else {
						curr_blob = new ImageBlob();
						curr_blob.update(x, y);
						image_blobs.put(dest[i], curr_blob);
					}
				}
			}

			for (Entry<Integer, ImageBlob> entry : image_blobs.entrySet()){
				if (entry.getValue().size > width * height * MIN_FRACTIONAL_AREA){
					curr_blob = entry.getValue();
					curr_blob.computeCoords();
					curr_blob.setColor(FIDUCIAL_COLOR_CONVERSION.get(checkColor));

					x = (int) curr_blob.x;
					y = (int) curr_blob.y;

					red = Image.pixelByteToInt(src.getPixelRed(x, y));
					green = Image.pixelByteToInt(src.getPixelGreen(x, y));
					blue = Image.pixelByteToInt(src.getPixelBlue(x, y));

					if (fiducialClassifyPixel(red, green, blue, true) == checkColor){
						indFiducials.add(curr_blob);
					}
				}
			}
			//System.out.println("Number of components after color " + checkColor + " is " + indFiducials.size());
		}

		fiducials.clear();
		for (ImageBlob blob1 : indFiducials){
			for (ImageBlob blob2 : indFiducials){
				if (Math.abs(blob1.x - blob2.x) < width * SUFFICIENTLY_CLOSE &&
					blob1.color != blob2.color && blob1.y > blob2.y && Math.abs(blob1.y - blob2.y) <
					2 * (Math.sqrt(blob1.size / Math.PI) + Math.sqrt(blob2.size / Math.PI)) &&
					blob1.size < 3 * blob2.size && blob2.size < 3 * blob1.size){

					fiducials.add(new FiducialBlob(new Color[]{blob1.color, blob2.color}, (blob1.size + blob2.size) / 2, (int) blob1.x, (int) blob1.y));
				}
			}
		}
	}

	/* LESS IMPORTANT METHODS */

	// Currently does not separate connected components of different colors that touch
	public void update(Image src){
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

				write_values = COLOR_MAP.get(classifyPixel(red, green, blue, true));
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

		// Find all connected components exceeding a minimum label
		Map<Integer, ImageBlob> image_blobs = new HashMap<Integer, ImageBlob>();

		int x, y;
		int[] classify_data = new int[3];
		ImageBlob curr_blob;
		for (int i = 0; i < width * height; i++){
			x = i % width;
			y = (int) (i / width);

			red = Image.pixelByteToInt(src.getPixelRed(x, y));
			green = Image.pixelByteToInt(src.getPixelGreen(x, y));
			blue = Image.pixelByteToInt(src.getPixelBlue(x, y));

			classify_data = COLOR_MAP.get(classifyPixel(red, green, blue, true));
			if (classify_data[0] != 0 || classify_data[1] != 0 || classify_data[2] != 0){
				if (image_blobs.containsKey(dest[i])){
					image_blobs.get(dest[i]).update(x, y);
				} else {
					curr_blob = new ImageBlob();
					curr_blob.update(x, y);
					image_blobs.put(dest[i], curr_blob);
				}
			}
		}

		List<Entry<Integer, ImageBlob>> pairs = new ArrayList<Entry<Integer, ImageBlob>>(image_blobs.entrySet());

		Collections.sort(pairs, new Comparator<Map.Entry<Integer, ImageBlob>>(){
			public int compare( Map.Entry<Integer, ImageBlob> o1, Map.Entry<Integer, ImageBlob> o2 ) {
				if (o2.getValue().size == o1.getValue().size){
					return 0;
				} else if (o2.getValue().size > o1.getValue().size){
					return 1;
				} else {
					return -1;
				}
			}
		});

		fiducials.clear();
		blocks.clear();

		int color = 0;
		int nb_color = 0;
		int top_color = 0;
		int bottom_color = 0;
		int top_red, top_green, top_blue, bottom_red, bottom_green, bottom_blue;

		double radius;
		boolean plain_fiducial = false;
		boolean mixed_fiducial = false;
		FiducialBlob curr_fiducial;

		for (Entry<Integer, ImageBlob> entry : pairs){
			curr_blob = entry.getValue();
			curr_blob.computeCoords();
			if (curr_blob.size < width * height * MIN_FRACTIONAL_AREA){
				break;
			} else {
				x = (int) curr_blob.x;
				y = (int) curr_blob.y;

				red = Image.pixelByteToInt(src.getPixelRed(x, y));
				green = Image.pixelByteToInt(src.getPixelGreen(x, y));
				blue = Image.pixelByteToInt(src.getPixelBlue(x, y));
				color = classifyPixel(red, green, blue, true);

				radius = Math.sqrt(curr_blob.size / Math.PI);
				plain_fiducial = false;
				mixed_fiducial = false;

				if (y + 2 * radius < height){
					red = Image.pixelByteToInt(src.getPixelRed(x, (int) (y + 2 * radius)));
					green = Image.pixelByteToInt(src.getPixelGreen(x, (int) (y + 2 * radius)));
					blue = Image.pixelByteToInt(src.getPixelBlue(x, (int) (y + 2 * radius)));
					nb_color = classifyPixel(red, green, blue, true);

					if (nb_color > 0){
						plain_fiducial = true;

						/*for (int i = -2; i <= 2; i++){
							for (int j = -2; j <= 2; j++){
								red = Image.pixelByteToInt(src.getPixelRed(x + i, (int) (y + 2 * radius) + j));
								green = Image.pixelByteToInt(src.getPixelGreen(x + i, (int) (y + 2 * radius) + j));
								blue = Image.pixelByteToInt(src.getPixelBlue(x + i, (int) (y + 2 * radius) + j));

								if (nb_color != classifyPixel(red, green, blue, true)){
									plain_fiducial = false;
								}
							}
						}*/
					}
				}

				if (y + radius < height && y - radius > 0){
					top_red = Image.pixelByteToInt(src.getPixelRed(x, (int) (y + radius)));
					top_green = Image.pixelByteToInt(src.getPixelGreen(x, (int) (y + radius)));
					top_blue = Image.pixelByteToInt(src.getPixelBlue(x, (int) (y + radius)));
					top_color = classifyPixel(top_red, top_green, top_blue, true);

					bottom_red = Image.pixelByteToInt(src.getPixelRed(x, (int) (y + radius)));
					bottom_green = Image.pixelByteToInt(src.getPixelGreen(x, (int) (y + radius)));
					bottom_blue = Image.pixelByteToInt(src.getPixelBlue(x, (int) (y + radius)));
					bottom_color = classifyPixel(bottom_red, bottom_green, bottom_blue, true);

					if (top_color > 0 && bottom_color > 0 && top_color != bottom_color){
						mixed_fiducial = true;

						/*for (int i = -2; i <= 2; i++){
							for (int j = -2; j <= 2; j++){
								red = Image.pixelByteToInt(src.getPixelRed(x + i, (int) (y - radius) + j));
								green = Image.pixelByteToInt(src.getPixelGreen(x + i, (int) (y - radius) + j));
								blue = Image.pixelByteToInt(src.getPixelBlue(x + i, (int) (y - radius) + j));

								if (top_color != classifyPixel(red, green, blue, true)){
									mixed_fiducial = false;
								}
							}
						}

						for (int i = -2; i <= 2; i++){
							for (int j = -2; j <= 2; j++){
								red = Image.pixelByteToInt(src.getPixelRed(x + i, (int) (y + radius) + j));
								green = Image.pixelByteToInt(src.getPixelGreen(x + i, (int) (y + radius) + j));
								blue = Image.pixelByteToInt(src.getPixelBlue(x + i, (int) (y + radius) + j));

								if (bottom_color != classifyPixel(red, green, blue, true)){
									mixed_fiducial = false;
								}
							}
						}*/
					}
				}

				curr_blob.setColor(COLOR_CONVERSION.get(color));

				if (plain_fiducial){
					curr_fiducial = new FiducialBlob(new Color[]{COLOR_CONVERSION.get(color), COLOR_CONVERSION.get(nb_color)}, curr_blob.size, x, y);
					fiducials.add(curr_fiducial);
				} else if (mixed_fiducial){
					curr_fiducial = new FiducialBlob(new Color[]{COLOR_CONVERSION.get(top_color), COLOR_CONVERSION.get(bottom_color)}, curr_blob.size / 2, x, y);
					fiducials.add(curr_fiducial);
				} else {
					blocks.add(curr_blob);
				}
			}
		}
	}

	public void updateNoFiducials(Image src){
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

				write_values = COLOR_MAP.get(classifyPixel(red, green, blue, true));
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

		// Find all connected components exceeding a minimum label
		Map<Integer, ImageBlob> image_blobs = new HashMap<Integer, ImageBlob>();

		int x, y;
		int[] classify_data = new int[3];
		ImageBlob curr_blob;
		for (int i = 0; i < width * height; i++){
			x = i % width;
			y = (int) (i / width);

			red = Image.pixelByteToInt(src.getPixelRed(x, y));
			green = Image.pixelByteToInt(src.getPixelGreen(x, y));
			blue = Image.pixelByteToInt(src.getPixelBlue(x, y));

			classify_data = COLOR_MAP.get(classifyPixel(red, green, blue, true));
			if (classify_data[0] != 0 || classify_data[1] != 0 || classify_data[2] != 0){
				if (image_blobs.containsKey(dest[i])){
					image_blobs.get(dest[i]).update(x, y);
				} else {
					curr_blob = new ImageBlob();
					curr_blob.update(x, y);
					image_blobs.put(dest[i], curr_blob);
				}
			}
		}

		List<Entry<Integer, ImageBlob>> pairs = new ArrayList<Entry<Integer, ImageBlob>>(image_blobs.entrySet());

		Collections.sort(pairs, new Comparator<Map.Entry<Integer, ImageBlob>>(){
			public int compare( Map.Entry<Integer, ImageBlob> o1, Map.Entry<Integer, ImageBlob> o2 ) {
				if (o2.getValue().size == o1.getValue().size){
					return 0;
				} else if (o2.getValue().size > o1.getValue().size){
					return 1;
				} else {
					return -1;
				}
			}
		});

		blocks.clear();
		double radius;
		for (Entry<Integer, ImageBlob> entry : pairs){
			curr_blob = entry.getValue();
			curr_blob.computeCoords();
			if (curr_blob.size < width * height * MIN_FRACTIONAL_AREA){
				break;
			} else {
				blocks.add(curr_blob);
			}
		}
	}

	private void drawBlobs(Image src, Image dest){
		int width = src.getWidth();
		int height = src.getHeight();

		byte[] destpix = new byte[3 * width * height];
		GaussianBlur.apply(src.toArray(), destpix, width, height);
		src = new Image(destpix, width, height);

		int red, green, blue;
		int[] write_values;
		int color;

		for (int i = 0; i < width; i++){
			for (int j = 0; j < height; j++){
				red = Image.pixelByteToInt(src.getPixelRed(i, j));
				green = Image.pixelByteToInt(src.getPixelGreen(i, j));
				blue = Image.pixelByteToInt(src.getPixelBlue(i, j));

				color = classifyPixel(red, green, blue, false);
				if (color < 0){
					write_values = new int[]{-color, -color, -color};
				} else {
					write_values = COLOR_MAP.get(color);
				}
				dest.setPixel(i, j, (byte) write_values[0], (byte) write_values[1], (byte) write_values[2]);
			}
		}
	}

	private void drawFiducials(Image src, Image dest){
		int width = src.getWidth();
		int height = src.getHeight();

		byte[] destpix = new byte[3 * width * height];
		GaussianBlur.apply(src.toArray(), destpix, width, height);
		src = new Image(destpix, width, height);

		int red, green, blue;
		int[] write_values;
		int color;

		for (int i = 0; i < width; i++){
			for (int j = 0; j < height; j++){
				red = Image.pixelByteToInt(src.getPixelRed(i, j));
				green = Image.pixelByteToInt(src.getPixelGreen(i, j));
				blue = Image.pixelByteToInt(src.getPixelBlue(i, j));

				color = fiducialClassifyPixel(red, green, blue, false);
				if (color < 0){
					write_values = new int[]{-color, -color, -color};
				} else {
					write_values = FIDUCIAL_COLOR_MAP.get(color);
				}
				dest.setPixel(i, j, (byte) write_values[0], (byte) write_values[1], (byte) write_values[2]);
			}
		}
	}

	public void drawCentroid(Image src, Image dest, int color){
		if (blocks.size() > 0){
			int centroidX = (int) blocks.get(0).x;
			int centroidY = (int) blocks.get(0).y;
			int x, y;
			for (int i = -4; i < 5; i++){
				for (int j = -4; j < 5; j++){
					x = (int) (centroidX + i);
					y = (int) (centroidY + j);
					if (x >= 0 && y >= 0 && x < src.getWidth() && y < src.getHeight()){
						if (color == 1){
							dest.setPixel(x, y, (byte) 0, (byte) 255, (byte) 255);
						} else if (color == 2){
							dest.setPixel(x, y, (byte) 255, (byte) 0, (byte) 255);
						} else if (color == 3){
							dest.setPixel(x, y, (byte) 255, (byte) 255, (byte) 0);
						} else {
							dest.setPixel(x, y, (byte) 255, (byte) 255, (byte) 255);
						}
					}
				}
			}
		}
	}

	public void drawFiducialCentroids(Image src, Image dest){
		for (FiducialBlob fid : fiducials){
			int centroidX = (int) fid.x;
			int centroidY = (int) fid.y;
			int x, y;
			for (int i = -2; i <= 3; i++){
				for (int j = -2; j <= 3; j++){
					x = (int) (centroidX + i);
					y = (int) (centroidY + j);
					if (x >= 0 && y >= 0 && x < src.getWidth() && y < src.getHeight()){
						dest.setPixel(x, y, (byte) 0, (byte) 0, (byte) 0);
					}
				}
			}
		}
	}
}
