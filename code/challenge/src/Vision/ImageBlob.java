package Vision;

import java.awt.Color;

// add in logic to generate color based on center
// add in logic to generate distances to block or ball (different formula above and below horizon)

public class ImageBlob {
	// State Variables
	public int size;
	public int xtotal;
	public int ytotal;

	// Final Variables
	public double x = -1;
	public double y = -1;
	public Color color = Color.GREEN;

	public ImageBlob(){
		this.size = 0;
		this.xtotal = 0;
		this.ytotal = 0;
	}

	public void update(int xcoord, int ycoord){
		size++;
		xtotal += xcoord;
		ytotal += ycoord;
	}

	public void computeCoords(){
		x = ((double) xtotal) / size;
		y = ((double) ytotal) / size;
	}

	public void setColor(Color color){
		this.color = color;
	}

	public static double computeDistance(int pixel_area, double real_radius){
		double DISTANCE_GAIN = 0.0063368;
		double POST_GAIN = 1.06;

		double pixel_radius = Math.sqrt(pixel_area/Math.PI);
		double distance = real_radius / Math.sin(DISTANCE_GAIN * pixel_radius);

		return POST_GAIN * distance;
	}

	// returns bearing in radians
	public static double computeBearing(int x) {
		// Diagonal field of view is 63 degrees, width is 160, height is 120
		double diag_fov = 63.0 * (Math.PI / 180.0);
		double diag_pix = Math.sqrt(Math.pow(160, 2)+Math.pow(120, 2));
		double horiz_fov = 160.0 * (diag_fov / diag_pix);
		return horiz_fov * (((double) x / 160) - 0.5);
	}
}