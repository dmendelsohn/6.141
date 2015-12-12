package Vision;

public class Fiducial {
	// State Variables
	public int[] colors;
	public double distance;
	public int x;
	public int y;

	public Fiducial(int[] colors, double distance, int x, int y){
		this.colors = colors;
		this.distance = distance;
		this.x = x;
		this.y = y;
	}
}