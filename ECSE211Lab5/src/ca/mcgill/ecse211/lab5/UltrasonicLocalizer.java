package ca.mcgill.ecse211.lab5;

/** 
 * This class implements the ultrasonic localization.
 * @author Antonios Valkanas, Borui Tao
 * @version 1.0
 *  
 */

import lejos.hardware.Sound;

public class UltrasonicLocalizer extends Thread{
	private int choice;				// 0 for rising edge, 1 for falling edge
	private Navigation na;
	private Odometer odometer;

	private int distance;			// The distance between the robot and the wall

	private double firstAngle;		// The angle where the robot detect the wall for the first time
	private double secondAngle;		// The angle where the robot detect the wall for the second time
	private double correctedTheta;	// The angle that the robot is away from the zero angle

	private boolean firstPointDetected;	 // Used to prevent the robot from detecting a point 2 times
	private boolean secondPointDetected; // Used to prevent the robot from detecting a point 2 times

	private double locationX;	// current X
	private double locationY;	// current Y
	private double locationTheta;

	//This is the constructor
	public UltrasonicLocalizer(int choice, Navigation navigation, Odometer odometer) {
		this.na = navigation;
		this.odometer = odometer;

		this.distance = 0;
		this.correctedTheta = 0;
		this.firstPointDetected = false;
		this.secondPointDetected = false;

		this.locationX = 0;
		this.locationY = 0;
	}

	/** 
	 * This method updates the distance
	 * 
	 * @param  distance   The distance from the object detected by the Ultrasonic Sensor
	 */
	public void processUSData(int distance) {
		this.distance = distance;
	}

	/** 
	 * This method runs the thread 
	 */
	public void run(){
		na.makeTurn(360);

		while (distance == 1){

		}
		if (choice == 0) risingEdge();
		else fallingEdge();
	}

	/**
	 * This method localizes the robot in the falling edge mode
	 * The ultrasonic sensor in front of the robot keeps detecting the distance between the wall and the robot
	 * When the ultrasonic sensor detects the falling edge, the robot records the turning angle, reverses the turning 
	 * direction and keeps turning until it finds the next falling edge. 
	 * 
	 * Because the noise margin exists, we record two turning angles.The first one is when the distance is greater than the sum of
	 * distance threshold and the noise margin (d+k); The second one is when it is less than the difference between them (d-k).
	 * Every time the falling edge is detected when the robot gets the two readings and the turning angle is the average of these two 
	 * readings.
	 *
	 */
	private void fallingEdge(){
		double theta1 = 0;
		double theta2 = 0;
		// Detect the first angle where the robot sees a wall from the distance. 
		// wait until the distance is greater than the threshold + noise margin + 1
		while(distance > (ZiplineLab.DISTANCE_THRESHOLD + ZiplineLab.NOISE_MARGIN+1)){
		}
		if(!firstPointDetected){
			theta1 = odometer.getTheta();
			// The second theta is detected if the distance is smaller than the threshold - noise margin - 1
			if (distance <= (ZiplineLab.DISTANCE_THRESHOLD - ZiplineLab.NOISE_MARGIN - 1)){
				theta2 = odometer.getTheta();
				Sound.playNote(Sound.FLUTE, 440, 250);
			}
			// the first angle is the average of the two thetas
			firstAngle = (theta1 + theta2) / 2;
			firstPointDetected = true;
		}
		//stop the motors and make it turn the opposite direction
		na.motorStop();
		na.makeTurn(-360);

		try {
			Thread.sleep(2000);
		} catch (Exception e) {
		}

		// Detect the second angle where the robot sees a wall from the distance. 
		// wait until the distance is greater than the threshold + noise margin + 1
		while(distance > (ZiplineLab.DISTANCE_THRESHOLD + ZiplineLab.NOISE_MARGIN+1)){
		}
		if (!secondPointDetected){
			theta1 = odometer.getTheta();

			// The second theta is detected if the distance is smaller than the threshold - noise margin - 1
			if (distance <= (ZiplineLab.DISTANCE_THRESHOLD - ZiplineLab.NOISE_MARGIN - 1)){
				theta2 = odometer.getTheta();
				Sound.playNote(Sound.FLUTE, 440, 250);
			}
			secondPointDetected = true;
			// the second angle is the average of the two thetas
			secondAngle = (theta1 + theta2) / 2;
		}
		na.motorStop();
		correctAngleAndLocation();
	}


	/**
	 * This method localizes the robot in the rising edge mode. 
	 * 
	 * Similar to the falling edge, we record two turning angles and record their average value to be the turning angle. 
	 * The first one is when the distance is less than the difference of distance threshold and the noise margin (d-k); 
	 * The second one is when it is greater than the sum between them (d+k).
	 * Every time the rising edge is detected when the robot gets the two readings and the turning angle is the average of these two 
	 * readings.
	 *
	 */
	private void risingEdge(){
		double theta1 = 0;
		double theta2 = 0;

		// Detect the first angle where the robot sees a wall from the distance. 
		// wait until the distance is smaller than the threshold - noise margin - 1
		while(distance < (ZiplineLab.DISTANCE_THRESHOLD - ZiplineLab.NOISE_MARGIN-1)){
		}
		if (!firstPointDetected){
			theta1 = odometer.getTheta();
			// The second theta is detected if the distance is greater than the threshold + noise margin + 1
			if (distance >= (ZiplineLab.DISTANCE_THRESHOLD + ZiplineLab.NOISE_MARGIN + 1)){
				theta2 = odometer.getTheta();
				Sound.playNote(Sound.FLUTE, 440, 250);
			}
			// the first angle is the average of the two thetas
			firstAngle = (theta1 + theta2) / 2;
			firstPointDetected = true;
		}
		//stop the motors and make it turn the opposite direction
		na.motorStop();
		na.makeTurn(-360);

		try {
			Thread.sleep(2000);
		} catch (Exception e) {
		}

		// Detect the second angle where the robot sees a wall from the distance. 
		// wait until the distance is smaller than the threshold - noise margin - 1
		while(distance < (ZiplineLab.DISTANCE_THRESHOLD - ZiplineLab.NOISE_MARGIN-1)){
		}
		if(!secondPointDetected){
			theta1 = odometer.getTheta();
			// wait until the distance is greater than the threshold + noise margin + 1
			if (distance >= (ZiplineLab.DISTANCE_THRESHOLD + ZiplineLab.NOISE_MARGIN + 1)){
				theta2 = odometer.getTheta();
				Sound.playNote(Sound.FLUTE, 440, 250);
			}
			// the second angle is the average of the two thetas
			secondAngle = (theta1 + theta2) / 2;
			secondPointDetected = true;
		}
		na.motorStop();
		correctAngleAndLocation();
	}

	/**
	 * This method gets the two turning angles either from the falling edge mode or the rising edge mode. 
	 * First it measures the approximate location of the robot. This measurements are applied in the main class
	 * to navigate the robot to the point (0,0).
	 * Then it calculates the corrected angle according to the two turning angles and the chosen mode
	 * Finally, it set the odometer Theta to be the calculated value and turn the robot to the 0 degree direction. 
	 *
	 */
	public void correctAngleAndLocation(){		  
		locationTheta = (firstAngle + (360 - secondAngle) - 90) / 2 ;
		if (choice == 0){
			locationX = ZiplineLab.DISTANCE_THRESHOLD * Math.cos(Math.toRadians(locationTheta));
			locationY = ZiplineLab.DISTANCE_THRESHOLD * Math.cos(Math.toRadians(locationTheta));
		}else if (choice == 1){
			locationX = ZiplineLab.DISTANCE_THRESHOLD * Math.sin(Math.toRadians(locationTheta));
			locationY = ZiplineLab.DISTANCE_THRESHOLD * Math.sin(Math.toRadians(locationTheta));
		}

		firstAngle = ((firstAngle % 360) + 360) % 360;
		secondAngle = ((secondAngle % 360) + 360) % 360;

		if (choice == 0) {
			correctedTheta = 45 - ((firstAngle + secondAngle) / 2);
		} else {
			correctedTheta = 225 - ((firstAngle + secondAngle) / 2);
		}

		// Get the current odometer reading theta, add its value to the calculated theta correction. Then set the result
		// to the odometer. 
		Double t = odometer.getTheta();
		correctedTheta += t;
		correctedTheta = ((correctedTheta % 360) + 360) % 360;
		odometer.setTheta(correctedTheta);	
		na.makeMinimumTurn(-correctedTheta);
	}

	/**
	 * This method returns the value of the distance.
	 * The if statement is there to put a maximum in the distance value in case the US sensor
	 * does not detect anything. This is done to help us collect and graph distance data
	 * since if the sensor does not detect anything it will return a value greater than 200,000 which is obviously incorrect.
	 * The maximum of the distance value is not used for anything else so this is not "hard-coding" to control the robot movement.
	 * @return 	distance value
	 */
	public int readUSDistance() {
		if (this.distance > 300)
			this.distance = 300;
		return this.distance;
	}

	/**
	 * This method returns the value of the x coordinate
	 * @return 	x value
	 */
	public double getLocX(){
		return this.locationX;
	}
	/**
	 * This method returns the value of the y coordinate
	 * @return 	y value
	 */
	public double getLocY(){
		return this.locationY;
	}

}

