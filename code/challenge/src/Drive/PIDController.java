package Drive;

public class PIDController {
	double gainP, gainI, gainD, maxP, maxI, maxD, prevTime;
	double prevError = 0;
	double integral = 0;

	public PIDController(double gainP, double gainI, double gainD, double maxP, double maxI, double maxD){
		this.gainP = gainP;
		this.gainI = gainI;
		this.gainD = gainD;

		this.maxP = maxP;
		this.maxI = maxI;
		this.maxD = maxD;

		prevTime = System.currentTimeMillis();
	}

	public double update(double error){
		double deriv = (error - prevError) / (System.currentTimeMillis() - prevTime);
		integral += (System.currentTimeMillis() - prevTime) * error;

		double errorM = Math.max(Math.min(error, maxP), -maxP);
		double integralM = Math.max(Math.min(integral, maxI), -maxI);
		double derivM = Math.max(Math.min(deriv, maxD), -maxD);

		double out = gainP * errorM + gainI * integralM + gainD * derivM;

		prevTime = System.currentTimeMillis();
		prevError = error;

		return out;
	}
}
