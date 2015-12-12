package Vision;

import java.awt.Color;
import java.awt.geom.*;
import java.util.*;

import MotionPlanning.*;
import MapFiles.*;

public class AngleLocalization {
	// Constants
	private final int NUM_PARTICLES = 1000;
	private final double DIST_SIGMA = 5.0;
	private final double ROT_SIGMA = 0.1;
	private final double INITIAL_NOISE = 0.05;
	private final double DIST_GAIN = 1.0;
	private final double NORMALIZATION = 1.0;

	// Odometry Variables
	private double prev_odo_x;
	private double prev_odo_y;
	private double prev_odo_theta;

	// C-Space Variables
	List<PolygonObstacle> cSpaceObstacles;
	Rectangle2D.Double cSpaceWorld;
	Point2D.Double robotStart;
	Fiducial[] fiducialPairs;

	// Particle Filter Variables
	List<OdoPoint> particles;
	List<Double> probabilities;
	OdoPoint localPoint;

	public AngleLocalization(List<PolygonObstacle> cSpaceObstacles, Rectangle2D.Double cSpaceWorld, Point2D.Double robotStart,
		Fiducial[] fiducialPairs){

		this.cSpaceObstacles = cSpaceObstacles;
		this.cSpaceWorld = cSpaceWorld;
		this.robotStart = robotStart;
		this.fiducialPairs = fiducialPairs;

		particles = new ArrayList<OdoPoint>();
		probabilities = new ArrayList<Double>();

		double delta_x = 0;
		double delta_y = 0;
		double delta_theta = 0;
		for (int i = 0; i < NUM_PARTICLES; i++){
			delta_x = cSpaceWorld.getWidth() * INITIAL_NOISE * (1 - 2 * Math.random());
			delta_y = cSpaceWorld.getHeight() * INITIAL_NOISE * (1 - 2 * Math.random());
			delta_theta = 2 * Math.PI * INITIAL_NOISE * (1 - 2 * Math.random());
			particles.add(new OdoPoint(robotStart.x + delta_x, robotStart.y + delta_y, delta_theta));
			probabilities.add(1.0 / NUM_PARTICLES);
		}

		prev_odo_x = robotStart.x;
		prev_odo_y = robotStart.y;
		prev_odo_theta = 0;
		localPoint = new OdoPoint(robotStart.x, robotStart.y, 0);
	}

	public OdoPoint localize(double odo_x, double odo_y, double odo_theta, List<FiducialBlob> fiducials){
		double x_disp = odo_x - prev_odo_x;
		double y_disp = odo_y - prev_odo_y;
		double theta_disp = odo_theta - prev_odo_theta;

		prev_odo_x = odo_x;
		prev_odo_y = odo_y;
		prev_odo_theta = odo_theta;

		localPoint = new OdoPoint(localPoint.x + x_disp, localPoint.y + y_disp, localPoint.theta + theta_disp);

		List<FiducialBlob> revFiducials = new ArrayList<FiducialBlob>();
		double mapx, mapy;
		Color[] colors = new Color[2];
		Fiducial mapFid;

		double mapbearing;
		double localAngle = localPoint.theta;
		localAngle = localAngle - ((int) (localAngle / (2 * Math.PI)));
		if (localAngle > Math.PI){
			localAngle -= 2 * Math.PI;
		}

		List<OdoPoint> new_particles = new ArrayList<OdoPoint>();
		List<Double> new_probs = new ArrayList<Double>();
		List<Double> cumulative_probs = new ArrayList<Double>();

		OdoPoint curr_point;
		double curr_prob, distance, fid_distance, fid_bearing;
		double probsum = 0;

		// Filter out unreasonable fiducials
		for (FiducialBlob fid : fiducials){
			colors[0] = fid.colors[0];
			colors[1] = fid.colors[1];

			if (findFiducial(colors[0], colors[1]) != null){
				mapFid = findFiducial(colors[0], colors[1]);
				mapx = mapFid.getPosition().x;
				mapy = mapFid.getPosition().y;

				mapbearing = Math.atan2(mapy - localPoint.y, mapx - localPoint.x);

				distance = Math.sqrt((localPoint.x - mapx) * (localPoint.x - mapx) 
					+ (localPoint.y - mapy) * (localPoint.y - mapy));

				fid_distance = ImageBlob.computeDistance(fid.size, mapFid.getTopSize());

				System.out.println("Localization: map and camera bearing = " + mapbearing + ", " + localAngle);
				System.out.println("Localization: map and camera distance = " + distance + ", " + fid_distance);

				if ((Math.abs(mapbearing - localAngle) < Math.PI / 6 || Math.abs(mapbearing - localAngle + 2 * Math.PI) < Math.PI / 6
					|| Math.abs(mapbearing - localAngle - 2 * Math.PI) < Math.PI / 6) && fid_distance < distance * 1.3 && distance < fid_distance * 1.3){

					System.out.println("Localization: added fiducial");
					revFiducials.add(fid);
				} else {
					System.out.println("Localization: eliminated fiducial from bearing and distance");
				}
			} else {
				System.out.println("Localization: eliminated fiducial from colors");
			}
		}

		/*// Use Odometry if no fiducials
		if (revFiducials.size() == 0){
			System.out.println("Localization: no fiducials to localize with");
			particles.clear();
			OdoPoint curr_point;
			for (int i = 0; i < NUM_PARTICLES; i++){
				curr_point = particles.get(i);
				particles.add(new OdoPoint(curr_point.x + x_disp, curr_point.y + y_disp, curr_point.theta + theta_disp));
			}
			return localPoint;
		}*/

		for (int i = 0; i < NUM_PARTICLES; i++){
			// Motion Update
			curr_point = particles.get(i);
			new_particles.add(new OdoPoint(curr_point.x + x_disp, curr_point.y + y_disp, curr_point.theta + theta_disp));

			// Sensor Update
			curr_prob = 1;
			for (FiducialBlob fid : revFiducials){
				colors[0] = fid.colors[0];
				colors[1] = fid.colors[1];

				mapFid = findFiducial(colors[0], colors[1]);
				mapx = mapFid.getPosition().x;
				mapy = mapFid.getPosition().y;

				distance = Math.sqrt((curr_point.x + x_disp - mapx) * (curr_point.x + x_disp - mapx) 
					+ (curr_point.y + y_disp - mapy) * (curr_point.y + y_disp - mapy));

				fid_distance = ImageBlob.computeDistance(fid.size, mapFid.getTopSize());

				curr_prob *= NORMALIZATION * (Math.exp(-0.5 *(fid_distance - distance) * (fid_distance - distance) / DIST_SIGMA) 
					/ Math.sqrt(2 * Math.PI * DIST_SIGMA * DIST_SIGMA));

				fid_bearing = curr_point.theta + theta_disp - ImageBlob.computeBearing(fid.x);
				fid_bearing = fid_bearing - ((int) (fid_bearing / (2 * Math.PI)));
				if (fid_bearing > Math.PI){
					fid_bearing -= 2 * Math.PI;
				}
				mapbearing = Math.atan2(mapy - curr_point.y - y_disp, mapx - curr_point.x - x_disp);

				curr_prob *= NORMALIZATION * (Math.exp(-0.5 *(mapbearing - fid_bearing) * (mapbearing - fid_bearing) / ROT_SIGMA) 
					/ Math.sqrt(2 * Math.PI * ROT_SIGMA * ROT_SIGMA));

			}

			probsum += curr_prob;
			new_probs.add(curr_prob);
			cumulative_probs.add(probsum);
		}

		if (revFiducials.size() > 0){
			System.out.println("Localization: localizing with fiducials");
			particles.clear();
			double rand_val;
			int index;
			int max_index = 0;
			for (int i = 0; i < NUM_PARTICLES; i++){
				rand_val = probsum * Math.random();
				index = Collections.binarySearch(cumulative_probs, rand_val);
				if (index < 0){
					index = - (index + 1);
				}

				particles.add(new_particles.get(index));

				if (new_probs.get(max_index) < new_probs.get(i)){
					max_index = i;
				}
			}

			localPoint = new_particles.get(max_index);
		} else {
			System.out.println("Localization: no fiducials to localize with");
			particles = new_particles;
		}

		return localPoint;
	}

	public OdoPoint encoderUpdate(double odo_x, double odo_y, double odo_theta){
		 return new OdoPoint(localPoint.x + odo_x - prev_odo_x, localPoint.y + odo_y - prev_odo_y, localPoint.theta + odo_theta - prev_odo_theta);
	}

	private Fiducial findFiducial(Color bottom, Color top){
		for (Fiducial fid : fiducialPairs){
			if (fid.getBottomColor() == bottom && fid.getTopColor() == top){
				return fid;
			}
		}
		return null;
	}
}
