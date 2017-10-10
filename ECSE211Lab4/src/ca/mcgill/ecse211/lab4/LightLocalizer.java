package ca.mcgill.ecse211.lab4;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;
import lejos.hardware.Sound;

public class LightLocalizer extends Thread {
	private static final long CORRECTION_PERIOD = 11;
	private Odometer odometer;
	private Navigation navigation;
	private static EV3ColorSensor lightSensor = new EV3ColorSensor(LocalEV3.get().getPort("S2")); //setup light sensor
	private static SampleProvider colorSensor = lightSensor.getMode("Red"); 
	private float[] lightValue; //saves the sensor data
	private float prevLightValue; // previous value of light sensor
	private int lineCounter; // keeps track of the amount of lines crossed
	private final int DISTANCE = 13; //distance from light sensor to center of rotation

	// constructor
	public LightLocalizer(Odometer odometer, Navigation navigation) {
		this.navigation = navigation;
		this.odometer = odometer;
		lightValue = new float[colorSensor.sampleSize()];
		prevLightValue = lightValue[0];
		lineCounter = 0;

	}

	// run method (required for Thread)
	public void run() {
		long correctionStart = 0;
		long correctionEnd;
		double thetaXminus = 0, thetaXplus = 0, thetaYplus = 0, thetaYminus = 0;
		colorSensor.fetchSample(lightValue,0);
		prevLightValue = lightValue[0]; 

		navigation.makeTurn(360);

		while (navigation.isNavigating()) {
			correctionStart = System.currentTimeMillis();
			colorSensor.fetchSample(lightValue,0);


			if ((prevLightValue - lightValue[0])/prevLightValue > 0.135) { //Line detected
				lineCounter++;
				System.out.println("line detected" + prevLightValue + "  " + lightValue[0]);
				double curTheta = odometer.getTheta();
				System.out.println("Current Theta and the line counter is:" + curTheta + " " + lineCounter);
				Sound.playNote(Sound.FLUTE, 440, 250); // sound to let us know robot sees the line

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
					navigation.motorStop();
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

		

		double thetaX = (thetaXplus - thetaXminus)/2;
		double thetaY = (thetaYminus - thetaYplus)/2;
		System.out.println("all the angles are: " + thetaXminus + " " + thetaYplus + " " + thetaXminus + " " + thetaYminus );

		double positionX = (-1 * DISTANCE) * Math.abs(Math.cos(Math.toRadians(thetaY)));
		double positionY = (-1 * DISTANCE) * Math.abs(Math.cos(Math.toRadians(thetaX)));
		System.out.println("positionX is" + positionX);
		System.out.println("positionY is" + positionY);

		odometer.setX(positionX);
		odometer.setY(positionY);

		navigation.travelTo(0, 0);

		while (navigation.isNavigating()) {
			//Wait to get to point
			//System.out.println("\t waiting");
		}
		navigation.makeCorrectedTurn(-odometer.getTheta() + thetaYminus - 270 - thetaY); //I think this will take care of theta, if the test fails I will implement the math from tutorial that I proved on your notebook yesterday
	}
}