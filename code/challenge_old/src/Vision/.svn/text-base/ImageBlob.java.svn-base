package Vision;

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
	public int color = -1;
	public double distance = -1;
	public double bearing = -1;

	// Measured Constants
	private final double FIDUCIAL_RADIUS = 4.1;
	private final double BLOCK_RADIUS = 2.0;
	private final double DISTANCE_GAIN = 0.0063368;
	private final double YCOORD_HORIZON = 120.0;
	private final double FOCAL_PLANE_DISTANCE = 107.0;
	private final double WIDTH = 320;
	private final double HEIGHT = 240;

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

	public void finalize(int color, boolean is_fiducial){
		this.color = color;

		double radius;
		if (is_fiducial){
			radius = FIDUCIAL_RADIUS;
		} else {
			radius = BLOCK_RADIUS;
		}

		double pixel_radius = Math.sqrt(size/Math.PI);
		distance = radius / Math.sin(DISTANCE_GAIN * pixel_radius);
		bearing = Math.atan2(x - (WIDTH / 2.0), FOCAL_PLANE_DISTANCE);
	}
}