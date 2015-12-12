package Vision;

import java.awt.Color;
import java.awt.geom.*;
import java.util.*;

import MotionPlanning.*;
import MapFiles.*;

public class Localization {
	// Constants
	private final int NUM_PARTICLES = 500;
	private final double SIGMA = 0.1;
	private final double INITIAL_NOISE = 0.05;
	private final double DIST_GAIN = 1.0;

	// Odometry Variables
	private double prev_odo_x = 0;
	private double prev_odo_y = 0;
	//private double prev_odo_theta = 0;

	// C-Space Variables
	List<PolygonObstacle> cSpaceObstacles;
	Rectangle2D.Double cSpaceWorld;
	Point2D.Double robotStart;
	Map<Color[], Fiducial> fiducialPairs;

	// Particle Filter Variables
	List<Point2D.Double> particles;
	List<Double> probabilities;
	Point2D.Double localPoint;

	public Localization(List<PolygonObstacle> cSpaceObstacles, Rectangle2D.Double cSpaceWorld, Point2D.Double robotStart,
		Map<Color[], Fiducial> fiducialPairs){

		this.cSpaceObstacles = cSpaceObstacles;
		this.cSpaceWorld = cSpaceWorld;
		this.robotStart = robotStart;
		this.fiducialPairs = fiducialPairs;

		particles = new ArrayList<Point2D.Double>();
		probabilities = new ArrayList<Double>();

		double delta_x = 0;
		double delta_y = 0;
		for (int i = 0; i < NUM_PARTICLES; i++){
			delta_x = cSpaceWorld.getWidth() * INITIAL_NOISE * (1 - 2 * Math.random());
			delta_y = cSpaceWorld.getHeight() * INITIAL_NOISE * (1 - 2 * Math.random());
			particles.add(new Point2D.Double(robotStart.x + delta_x, robotStart.y + delta_y));
			probabilities.add(1.0 / NUM_PARTICLES);
		}
	}

	//public Point2D.Double localize(double odo_x, double odo_y, double odo_theta, List<Fiducial> fiducials){
	public Point2D.Double localize(double odo_x, double odo_y, List<FiducialBlob> fiducials){
		double x_disp = odo_x - prev_odo_x;
		double y_disp = odo_y - prev_odo_y;

		prev_odo_x = odo_x;
		prev_odo_y = odo_y;

		if (fiducials.size() == 0){
			System.out.println("Localization: no fiducials to localize with");
			particles.clear();
			Point2D.Double curr_point;
			for (int i = 0; i < NUM_PARTICLES; i++){
				// Motion Update
				curr_point = particles.get(i);
				particles.add(new Point2D.Double(curr_point.x + x_disp, curr_point.y + y_disp));
			}
			localPoint = new Point2D.Double(localPoint.x + x_disp, localPoint.y + y_disp);
			return localPoint;
		}

		//double theta_disp = odo_theta - prev_odo_theta;
		//prev_odo_theta = odo_theta;

		List<Point2D.Double> new_particles = new ArrayList<Point2D.Double>();
		List<Double> new_probs = new ArrayList<Double>();
		List<Double> cumulative_probs = new ArrayList<Double>();

		Point2D.Double curr_point;
		double curr_prob, distance, fid_distance, mapx, mapy;
		double probsum = 0;
		Color[] colors = new Color[2];
		Fiducial mapFid;
		for (int i = 0; i < NUM_PARTICLES; i++){
			// Motion Update
			curr_point = particles.get(i);
			new_particles.add(new Point2D.Double(curr_point.x + x_disp, curr_point.y + y_disp));

			// Sensor Update
			curr_prob = 1;
			for (FiducialBlob fid : fiducials){
				colors[0] = fid.colors[0];
				colors[1] = fid.colors[1];

				mapFid = fiducialPairs.get(colors);
				mapx = mapFid.getPosition().x;
				mapy = mapFid.getPosition().y;

				distance = Math.sqrt((curr_point.x + x_disp - mapx) * (curr_point.x + x_disp - mapx) 
					+ (curr_point.y + y_disp - mapy) * (curr_point.y + y_disp - mapy));

				fid_distance = ImageBlob.computeDistance(fid.size, mapFid.getTopSize());

				curr_prob *= (Math.exp(-0.5 *(fid_distance - distance) * (fid_distance - distance) / SIGMA) 
					/ Math.sqrt(2 * Math.PI * SIGMA * SIGMA));

			}

			probsum += curr_prob;
			new_probs.add(curr_prob);
			cumulative_probs.add(probsum);
		}

		particles.clear();
		double rand_val;
		int index;
		int max_index = 0;
		for (int i = 0; i < NUM_PARTICLES; i++){
			rand_val = probsum * Math.random();
			index = Math.abs(Collections.binarySearch(cumulative_probs, rand_val));

			particles.add(new_particles.get(index));

			if (new_probs.get(max_index) < new_probs.get(i)){
				max_index = i;
			}
		}

		localPoint = new_particles.get(max_index);

		return localPoint;
	}

	public Point2D.Double encoderUpdate(double odo_x, double odo_y){
		localPoint = new Point2D.Double(localPoint.x + odo_x - prev_odo_x, localPoint.y + odo_y - prev_odo_y);
		return localPoint;
	}
}
