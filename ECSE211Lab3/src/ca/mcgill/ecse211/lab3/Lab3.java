package ca.mcgill.ecse211.lab3;

/** This is the main class. It makes the robot run
 * @author Antonios Valkanas, Borui Tao
 * @version 1.0
 * 
 */

import lejos.hardware.Button;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Lab3 {
	private static Odometer odometer = null;

	public static final EV3MediumRegulatedMotor sensorMotor =
			new EV3MediumRegulatedMotor(LocalEV3.get().getPort("B"));

	public static final EV3LargeRegulatedMotor leftMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));

	public static final EV3LargeRegulatedMotor rightMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

	private static final Port usPort = LocalEV3.get().getPort("S1");

	//All the constants
	public static final int FORWARD_SPEED = 190;
	public static final int ROTATE_SPEED = 100;
	public static final double SQUARE_LENGTH = 30.48; 
	public static final double RADIUS = 2.12;
	public static final double TRACK = 11.7;
	
	public static void main(String[] args) {    
		// clear the display
		@SuppressWarnings("resource")							    // Because we don't bother to close this resource
		SensorModes ultrasonicSensor = new EV3UltrasonicSensor(usPort);		// usSensor is the instance
		SampleProvider usDistance = ultrasonicSensor.getMode("Distance");	// usDistance provides samples from this instance
		float[] usData = new float[1];		// usData is the buffer in which data are returned
		UltrasonicPoller usPoller = null;									// the selected controller on each cycle

		final TextLCD t = LocalEV3.get().getTextLCD();
		odometer = new Odometer(leftMotor, rightMotor);
		
		t.clear();
		t.drawString("Press any button.", 0, 0);
		Button.waitForAnyPress();
		
		//Start the odometer and the display
		//Wait for 5 seconds
		try { 
			Thread.sleep(5000);
		} catch (Exception e) {
		}
		
		Navigation na = new Navigation(sensorMotor,odometer,leftMotor, rightMotor);
		OdometryDisplay odometryDisplay = new OdometryDisplay(odometer, t, na);
		usPoller = new UltrasonicPoller(usDistance, usData, na);
		odometer.start();
		odometryDisplay.start();
		na.start();
		usPoller.start();
        
        while (Button.waitForAnyPress() != Button.ID_ESCAPE);

		System.exit(0);
	}
}
