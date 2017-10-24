package ca.mcgill.ecse211.lab5;
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

public class LightLocalizer extends Thread {
	private static final long CORRECTION_PERIOD = 10;
	private Odometer odometer;
	private Navigation navigation;
	private static EV3ColorSensor lightSensor = new EV3ColorSensor(LocalEV3.get().getPort("S4")); //setup light sensor
	private static SampleProvider colorSensor = lightSensor.getMode("Red"); 
	private float[] lightValue; //saves the sensor data
	private float prevLightValue; // previous value of light sensor
	private int lineCounter; // keeps track of the amount of lines crossed
	private final double DISTANCE = 13; //distance from light sensor to center of rotation

	// constructor
	public LightLocalizer(Odometer odometer, Navigation navigation) {
		this.navigation = navigation;
		this.odometer = odometer;
		lightValue = new float[colorSensor.sampleSize()];
		prevLightValue = lightValue[0];
		lineCounter = 0;

	}
	
	public void run(){
		// run method

		doLightLocalization();
		while (navigation.isNavigating()) {
		}
		switch (ZiplineLab.cornerCounter) {
		case 0:
			odometer.setX(ZiplineLab.CORNERS[0][0]*ZiplineLab.SQUARE_LENGTH);
			odometer.setY(ZiplineLab.CORNERS[0][1]*ZiplineLab.SQUARE_LENGTH);
			odometer.setTheta(ZiplineLab.CORNERS[0][2]);
			System.out.println("         set 0!");
			break;
		case 1:
			odometer.setX(ZiplineLab.CORNERS[1][0]*ZiplineLab.SQUARE_LENGTH);
			odometer.setY(ZiplineLab.CORNERS[1][1]*ZiplineLab.SQUARE_LENGTH);
			odometer.setTheta(ZiplineLab.CORNERS[1][2]);
			System.out.println("         set 1!");
			break;
		case 2:
			odometer.setX(ZiplineLab.CORNERS[2][0]*ZiplineLab.SQUARE_LENGTH);
			odometer.setY(ZiplineLab.CORNERS[2][1]*ZiplineLab.SQUARE_LENGTH);
			odometer.setTheta(ZiplineLab.CORNERS[2][2]);
			System.out.println("         set 2!");
			break;
		case 3:
			odometer.setX(ZiplineLab.CORNERS[3][0]*ZiplineLab.SQUARE_LENGTH);
			odometer.setY(ZiplineLab.CORNERS[3][1]*ZiplineLab.SQUARE_LENGTH);
			odometer.setTheta(ZiplineLab.CORNERS[3][2]);
			System.out.println("         set 3!");
			break;
		} 
	}

	public void doLightLocalization() {
		long correctionStart = 0;	//used to keep track of correction period
		long correctionEnd;			//used to keep track of correction period
		double thetaXminus = 0, thetaXplus = 0, thetaYplus = 0, thetaYminus = 0; //value of theta at intersection of X+,X-,Y+,Y- axis
		colorSensor.fetchSample(lightValue,0); 
		prevLightValue = lightValue[0]; 

		navigation.makeTurn(360);

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

		// move to the origin
		navigation.travelTo(0, 0);

		//Wait to get to point
		while (navigation.isNavigating()) {
		}
		// apply angle correction - according to the tutorial formula
		navigation.makeMinimumTurn(-odometer.getTheta() + thetaYminus - 270 - thetaY); //I think this will take care of theta, if the test fails I will implement the math from tutorial that I proved on your notebook yesterday
	}
	
}