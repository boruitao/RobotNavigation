package ca.mcgill.ecse211.project;

/** LightSensor class
 * 	@author Raphael Di Piazza
 * 	@version 1.0
 * 
 */

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

/**
 * This class is used to get the color of the bricks for flag recognition.
 */
public class LightSensor extends Thread {
	private static final long CORRECTION_PERIOD = 10;
	private static EV3ColorSensor lightSensor = new EV3ColorSensor(LocalEV3.get().getPort("S2")); //setup light sensor
	private static SampleProvider colorSensor = lightSensor.getMode("ColorID"); 
	private float[] lightValue; //saves the sensor data

	/**
	 * The constructor initializes the sensor's parameters.
	 */
	public LightSensor() {
		lightValue = new float[colorSensor.sampleSize()];

	}

	/**
	 * This method runs in a thread to update the value captured by the light sensor.
	 * This value will be treated in the CaptureFlag class to see if the robot detected the correct flag.
	 */
	public void run(){
		while(true){
			long correctionStart = 0;	//used to keep track of correction period
			long correctionEnd;			//used to keep track of correction period
			correctionStart = System.currentTimeMillis();
			colorSensor.fetchSample(lightValue,0);
			System.out.println("light value " + lightValue[0]);
			UltrasonicLocalizer.lightValue = lightValue[0];
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
	}
}
