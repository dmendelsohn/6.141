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

	public List<Fiducial> fiducials;
	public List<ImageBlob> blocks; 

	// Measured Constants
	private final double MIN_FRACTIONAL_AREA = 0.001;
	private final List<int[]> COLOR_MAP;

	public BlobTracking(int width, int height) {
		this.width = width;
		this.height = height;

		fiducials = new LinkedList<Fiducial>();
		blocks = new LinkedList<ImageBlob>();

		COLOR_MAP = new LinkedList<int[]>();
		COLOR_MAP.add(new int[]{0, 0, 0});
		COLOR_MAP.add(new int[]{255, 0, 0});
		COLOR_MAP.add(new int[]{0, 255, 0});
		COLOR_MAP.add(new int[]{0, 0, 255});
	}

	private void stepTiming() {
		double currTime = System.currentTimeMillis();
		stepCounter++;
		// if it's been a second, compute frames-per-second
		if (currTime - lastStepTime > 1000.0) {
			double fps = (double) stepCounter * 1000.0 / (currTime - lastStepTime);
			System.err.println("FPS: " + fps);
			stepCounter = 0;
			lastStepTime = currTime;
		}
	}

	public void apply(Image src, Image dest) {
		stepTiming(); // monitors the frame rate
		update(src);

		// Display RGB histogram
		//dest = Histogram.getHistogram(src, dest, false);

		// Display HSB histogram
		//dest = Histogram.getHistogram(src, dest, true);

		// Display Classification
		drawBlobs(src, dest);
		
		// Draw Centroid of largest component
		drawCentroid(src, dest, 0);
	}

	private int classifyPixel(int red, int green, int blue, boolean set_zero){
		float[] hsb = Color.RGBtoHSB(red, green, blue, null);
		float hue = hsb[0];
		float sat = hsb[1];

		if ((hue < 0.1 || hue > 0.9) && sat > 0.3){
			return 1;
		} else if (hue < 0.38 && hue > 0.28 && sat > 0.5){
			return 2;
		} else if (hue > 0.61 && hue < 0.71 && sat > 0.5){
			return 3;
		} else {
			if (set_zero){
				return 0;
			} else {
				return -(red + green + blue)/ 3;
			}
		}
	}

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
		int color, nb_color;
		double radius;
		boolean is_fiducial = false;
		Fiducial curr_fiducial;
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
				is_fiducial = false;

				if (y + 2 * radius < height){
					red = Image.pixelByteToInt(src.getPixelRed(x, (int) (y + 2 * radius)));
					green = Image.pixelByteToInt(src.getPixelGreen(x, (int) (y + 2 * radius)));
					blue = Image.pixelByteToInt(src.getPixelBlue(x, (int) (y + 2 * radius)));
					nb_color = classifyPixel(red, green, blue, true);

					if (nb_color > 0){
						is_fiducial = true;
					}

					for (int i = -1; i <= 1; i++){
						for (int j = -1; j <= 1; j++){
							red = Image.pixelByteToInt(src.getPixelRed(x + i, (int) (y + 2 * radius) + j));
							green = Image.pixelByteToInt(src.getPixelGreen(x + i, (int) (y + 2 * radius) + j));
							blue = Image.pixelByteToInt(src.getPixelBlue(x + i, (int) (y + 2 * radius) + j));

							if (nb_color != classifyPixel(red, green, blue, true)){
								is_fiducial = false;
							}
						}
					}
				}

				curr_blob.finalize(color, is_fiducial);

				if (is_fiducial){
					curr_fiducial = new Fiducial(new int[]{color, nb_color}, curr_blob.distance, x, y);
					fiducials.add(curr_fiducial);
				} else {
					blocks.add(curr_blob);
				}
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

	public void drawCentroid(Image src, Image dest, int color){
		if (fiducials.size() > 0){
			int centroidX = (int) fiducials.get(0).x;
			int centroidY = (int) fiducials.get(0).y;
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
}
