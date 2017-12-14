package ca.mcgill.ecse211.project;
/**
 * This class implements the Light Localization
 * @author Antonios Valkanas, Borui Tao
 * @version 1.0
 * 
 */
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;
import lejos.hardware.Sound;

/**
 * This class implements the light localization which
 * uses the color sensor to know where the robot is from the origin.
 */
public class LightLocalizer{
	/**
	 * The correction period so that the sensor does not update the value constently.
	 */
	private static final long CORRECTION_PERIOD = 9;
	/**
	 * The pointer to the odometer class.
	 */
	private Odometer odometer;
	/**
	 * The pointer to the Navigation class.
	 */
	private Navigation navigation;
	/**
	 * The pointer to the light sensor.
	 */
	private static EV3ColorSensor lightSensor = new EV3ColorSensor(LocalEV3.get().getPort("S4"));
	/**
	 * The sample provider for the light sensor.
	 */
	private static SampleProvider colorSensor = lightSensor.getMode("Red");
	/**
	 * The array that saves the sensor data.
	 */
	private float[] lightValue;
	/**
	 * The previous value of the light sensor.
	 */
	private float prevLightValue;
	/**
	 * The counter for the amount of lines crossed during the first localization.
	 */
	private int lineCounter;
	/**
	 * The distance from the light sensor to the center of rotation.
	 */
	private final double DISTANCE = 13;
	
	/**
	 * The corrected theta (the exact direction the robot is facing).
	 */
	public double currentTheta;
	/**
	 * The constructor for the class that initializes the odometer and navigation objects as well as the fields utilized
	 * to update the sensor's data.
	 * @param odometer		pointer to the Odometer
	 * @param navigation	pointer to the Navigation class
	 */
	public LightLocalizer(Odometer odometer, Navigation navigation) {
		this.navigation = navigation;
		this.odometer = odometer;
		lightValue = new float[colorSensor.sampleSize()];
		prevLightValue = lightValue[0];
		lineCounter = 0;

	}

	/** 
	 * This method executes the first light localization.
	 */
	
	public void doLightLocalization() {
		long correctionStart = 0;	//used to keep track of correction period
		long correctionEnd;			//used to keep track of correction period
		double thetaXminus = 0, thetaXplus = 0, thetaYplus = 0, thetaYminus = 0; //value of theta at intersection of X+,X-,Y+,Y- axis
		colorSensor.fetchSample(lightValue,0); 
		prevLightValue = lightValue[0]; 

		navigation.makeTurn(360, false, true);

		while (navigation.isNavigating()) {
			correctionStart = System.currentTimeMillis();
			colorSensor.fetchSample(lightValue,0);

			// A primitive derivative calculation used to check the rate of change form point to point
			// for the light sensor data. If the rate of decrease is greater than 13% of the previous value we know we crossed a line
			if ((prevLightValue - lightValue[0])/prevLightValue > 0.135) {
				lineCounter++;
				double curTheta = odometer.getTheta();
				Sound.playNote(Sound.FLUTE, 440, 250); // sound to let us know robot sees the line
				
				// We are rotating clockwise so the first line intercept is X minus
				// the second at Y plus, third at X plus and the fourth Y minus
				
				switch (lineCounter) {
				case 1:
					thetaXminus = curTheta;
					break;
				case 2:
					thetaYplus = curTheta;
					break;
				case 3:
					thetaXplus = curTheta;
					break;
				case 4:
					thetaYminus = curTheta;
					navigation.motorStop(); //stop rotating after intercepting the fourth line (we already have all the information we want)
					break;
				}
			}
			prevLightValue = lightValue[0]; 

			// this ensures the light sensor check occurs only once every period
			correctionEnd = System.currentTimeMillis();
			if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
				try {
					Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here because it is not
					// expected that the odometry correction will be
					// interrupted by another thread
				}
			}
		} 

		
		// ThetaX and ThetaY divided by 2 calculation to find the error in X,Y and theta 
		// The validity of these formualae is shown in the tutorial slides.
		double thetaX = (thetaXplus - thetaXminus)/2;
		double thetaY = (thetaYminus - thetaYplus)/2;

		double positionX = (-1 * DISTANCE) * Math.abs(Math.cos(Math.toRadians(thetaY)));
		double positionY = (-1 * DISTANCE) * Math.abs(Math.cos(Math.toRadians(thetaX)));
		
		//apply coordinate correction
		odometer.setX(positionX);
		odometer.setY(positionY);

		System.out.println("The x is" + odometer.getX());
		System.out.println("The y is" + odometer.getY());
		System.out.println("The theta angle is" + odometer.getTheta());
		// move to the origin
		navigation.goToOrigin(0, 0);

		//Wait to get to point
		while (navigation.isNavigating()) {
		}
		// apply angle correction - according to the tutorial formula
		navigation.makeTurn((-odometer.getTheta() + thetaYminus - 270 - thetaY), true, false); //I think this will take care of theta, if the test fails I will implement the math from tutorial that I proved on your notebook yesterday
	}
	
	/**
	 * This method is used before the first localization to make sure the robot will hit the 4 axes during the light localization.
	 */
	public void adjustPosition(){
	//	odometer.setTheta(0);
		long correctionStart = 0;	//used to keep track of correction period
		long correctionEnd;			//used to keep track of correction period

		colorSensor.fetchSample(lightValue,0); 
		prevLightValue = lightValue[0]; 
	
		int counter = 0;
		while(counter < 2){
			if (counter == 1) navigation.makeTurn(90, true, false);
			navigation.motorMoveForward();
			while (navigation.isNavigating()) {
				correctionStart = System.currentTimeMillis();
				colorSensor.fetchSample(lightValue,0);

				// A primitive derivative calculation used to check the rate of change form point to point
				// for the light sensor data. If the rate of decrease is greater than 13% of the previous value we know we crossed a line
				if ((prevLightValue - lightValue[0])/prevLightValue > 0.135) {
					Sound.playNote(Sound.FLUTE, 440, 250); 
					navigation.motorStop();

					double dis = (CaptureFlag.SQUARE_LENGTH / 3 + CaptureFlag.ROBOT_LENGTH);
					navigation.motorMoveBackward(dis);
					while(navigation.isNavigating()){

					}
					navigation.motorStop();
				}
				prevLightValue = lightValue[0]; 

				// this ensures the light sensor check occurs only once every period
				correctionEnd = System.currentTimeMillis();
				if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
					try {
						Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
					} catch (InterruptedException e) {
						// there is nothing to be done here because it is not
						// expected that the odometry correction will be
						// interrupted by another thread
					}
				}
			}
			counter++;
		}
	}
	
	/**
	 * This method executes all light localizations during the Navigation.
	 * It uses mostly the same structure as the doLightLocalization() method.
	 */
	public void midpointLocalization() {
		long correctionStart = 0;	//used to keep track of correction period
		long correctionEnd;			//used to keep track of correction period
		double thetaXminus = 0, thetaXplus = 0, thetaYplus = 0, thetaYminus = 0; //value of theta at intersection of X+,X-,Y+,Y- axis
		colorSensor.fetchSample(lightValue,0); 
		prevLightValue = lightValue[0]; 
		int lineCounter = 0;
		navigation.makeTurn(360, false, true);

		while (navigation.isNavigating()) {
			correctionStart = System.currentTimeMillis();
			colorSensor.fetchSample(lightValue,0);

			// A primitive derivative calculation used to check the rate of change form point to point
			// for the light sensor data. If the rate of decrease is greater than 13% of the previous value we know we crossed a line
			if ((prevLightValue - lightValue[0])/prevLightValue > 0.135) {
				lineCounter++;
				double curTheta = odometer.getTheta();
				Sound.playNote(Sound.FLUTE, 440, 250); // sound to let us know robot sees the line
				
				// We are rotating clockwise so the first line intercept is X minus
				// the second at Y plus, third at X plus and the fourth Y minus
				
				switch (lineCounter) {
				case 1:
					thetaXminus = curTheta;
					break;
				case 2:
					thetaYplus = curTheta;
					break;
				case 3:
					thetaXplus = curTheta;
					break;
				case 4:
					thetaYminus = curTheta;
					navigation.motorStop(); //stop rotating after intercepting the fourth line (we already have all the information we want)
					break;
				}
			}
			prevLightValue = lightValue[0]; 

			// this ensures the light sensor check occurs only once every period
			correctionEnd = System.currentTimeMillis();
			if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
				try {
					Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here because it is not
					// expected that the odometry correction will be
					// interrupted by another thread
				}
			}
		} 

		
		// ThetaX and ThetaY divided by 2 calculation to find the error in X,Y and theta 
		// The validity of these formualae is shown in the tutorial slides.
		double thetaX = (thetaXplus - thetaXminus)/2;
		double thetaY = (thetaYminus - thetaYplus)/2;

		double positionX = (-1 * DISTANCE) * Math.abs(Math.cos(Math.toRadians(thetaY)));
		double positionY = (-1 * DISTANCE) * Math.abs(Math.cos(Math.toRadians(thetaX)));
		
		//apply coordinate correction
		odometer.setX(positionX);
		odometer.setY(positionY);

		System.out.println("The x is" + odometer.getX());
		System.out.println("The y is" + odometer.getY());
		System.out.println("Theta before travel to" + odometer.getTheta());
		// move to the origin
		navigation.goToOrigin(0, 0);

		//Wait to get to point
		while (navigation.isNavigating()) {
		}
		// apply angle correction - according to the tutorial formula
		navigation.makeTurn((-odometer.getTheta() + thetaYminus - 270 - thetaY), true, false); //I think this will take care of theta, if the test fails I will implement the math from tutorial that I proved on your notebook yesterday
		setCurrentTheta();
	}
	
	/**
	 * This method gets the current theta from the odometer after a midpoint localization and corrects it to the exact value it is supposed to be.
	 */
	private void setCurrentTheta(){
		double curTheta = odometer.getTheta();
		System.out.println("the theta i am recalculating " + curTheta);
		double shiftTheta = curTheta % 90;
		if(shiftTheta > 45){
			this.currentTheta = curTheta + (90 - shiftTheta);
		}else{
			this.currentTheta = curTheta - shiftTheta;
		}
	}
}