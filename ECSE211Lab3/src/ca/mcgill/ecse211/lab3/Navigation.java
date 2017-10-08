package ca.mcgill.ecse211.lab3;

/** This class navigates the robot and avoids collisions.
 * @author Antonios Valkanas, Borui Tao
 * @version 1.0
 * 
 */

import java.util.ArrayList;
import java.util.Arrays;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;

public class Navigation extends Thread implements UltrasonicController{

	private Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private EV3MediumRegulatedMotor sensorMotor;

	private static boolean objectDetected = false; //A boolean variable that returns true if an object is detected

	private int distance;//Current distance from the object
	private int average; //average distance
	private int index; 
	private int[] lastFiveDistance = new int[5]; // Array of the last 5 US outputs
	private ArrayList<int[]> path = new ArrayList<int[]>();
	private int pointIndex = 0;
	
	//constructor
	public Navigation(EV3MediumRegulatedMotor sensorMotor, Odometer odometer, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
		this.odometer = odometer;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.sensorMotor = sensorMotor;

		this.index = 0;
		this.average = 0;
		Arrays.fill(lastFiveDistance, 30);


		//Test Map - Enter demo map here.
		path.add(new int[] {2, 1});
		path.add(new int[] {1, 1});
		path.add(new int[] {1, 2});
		path.add(new int[] {2, 0});

	}

	//run the thread
	public void run(){
		leftMotor.stop();
		rightMotor.stop();

		while (pointIndex < path.size()) {
			travelTo(path.get(pointIndex)[0], path.get(pointIndex)[1]);
			pointIndex++;
		}
	}
	/** 
	 * This method process the input data by averaging the previous data
	 * Because the US data tends to be inconsistent, we take the average of 
	 * 5 values from the sensor and set it equal to distance.
	 * To prevent high sensor values from skewing the average, we set values greater than 100 to 100.
	 * 
	 * @param  distance   The distance from the object detected by the Ultrasonic Sensor
	 */
	@Override
	public void processUSData(int distance) {
		distance = Math.min(distance, 100);
		this.index++;
		if(this.index == 5) {
			this.index = 0;
		}

		lastFiveDistance[index] = distance;

		average = 0;
		for (int i=0; i<lastFiveDistance.length; i++) { //Using a for loop to average the last five distance data
			average+=lastFiveDistance[i];
		}
		average = average / 5;

		this.distance = average;
	}

	/**
	 * This method makes the robot to travel to a specific location (x,y)
	 * First it calculates the heading angle that the robot must face
	 * Then it gets the minimum turning angle which the robot must turn to
	 * Afterwards it calls the method turnTo() to turn to that angle.
	 * 
	 * If the robot comes across a object, it turns to 90 degree clockwise and the distance sensor turns to 45 degree anti-clockwise
	 * Then it starts BangBang Controller and follows the side of the objects
	 * Final it passes the object and heads to the next point 
	 * 
	 * @param  x   The x-coordinate of the destination point 
	 * @param  y   The y-coordinate of the destination point 
	 */
	void travelTo(double x, double y) {    
		//Get the current readings from odometer
		double curX = odometer.getX();  
		double curY = odometer.getY();
		double curTheta = odometer.getTheta();

		//Convert the destination point to odometer coordinates 
		double destX = x * Lab3.SQUARE_LENGTH;
		double destY = y * Lab3.SQUARE_LENGTH;

		//Calculate the angle that the robot must face to rotate
		double headingTheta = Math.atan2(destX - curX, destY - curY); 
		//Convert the result to degree
		headingTheta = headingTheta * 180 / Math.PI; 	 
		//Make the result to within 0 and 359 degrees
		headingTheta = ((headingTheta) % 360 + 360) % 360; 

		//Make sure that the turning angle does not exceed 180 degree
		if (curTheta != headingTheta) { 
			double turningTheta = headingTheta - curTheta;

			if (turningTheta >= 180 && turningTheta <= 359) {
				turningTheta -= 360;
			}
			else if (turningTheta <= -180 && turningTheta >= -359) {
				turningTheta += 360;
			}
			//Turn to the resulting angle
			turnTo(turningTheta); 
		}
		// calculate the distance that the robot must travel to reach the next point
		double distance = Math.sqrt(Math.pow(destX - curX, 2) + Math.pow(destX - curY, 2)); 

		leftMotor.setSpeed(Lab3.FORWARD_SPEED);
		rightMotor.setSpeed(Lab3.FORWARD_SPEED);

		leftMotor.rotate(convertDistance(Lab3.RADIUS, distance), true);
		rightMotor.rotate(convertDistance(Lab3.RADIUS, distance), false);

		double lastTheta = 0;
		while(isNavigating()) {
			//If the distance from the object is small than a threshold, turn the robot to 90 degrees clockwise 
			// then turn the sensor heading to 45 degree anti-clockwise
			if(average <= 25 && !objectDetected) {
				objectDetected = true;
				lastTheta = odometer.getTheta();
				turnTo(90);
				sensorMotor.rotate(-45);
			}
			/**
			 * Start BangBang wall-following until the robot passes the object
			 * The robot passes the object when odometer's Theta is 90 degrees to the left of its heading angle
			 * when it first comes across the object
			 */
			else if (objectDetected) {
				if (lastTheta - 95 <= odometer.getTheta() && lastTheta - 85 >= odometer.getTheta()){
					System.out.println("Bang Bang Bang");
					if (average > 20 && average < 25) {
						System.out.println("between 25 and 25");
						leftMotor.setSpeed(Lab3.FORWARD_SPEED); // Start robot moving forward
						rightMotor.setSpeed(Lab3.FORWARD_SPEED);
						leftMotor.forward();
						rightMotor.forward();
					} 
					else if (average >= 25) {
						System.out.println("greater than 25");
						leftMotor.setSpeed(100); // Left turn
						rightMotor.setSpeed(Lab3.FORWARD_SPEED-20);//motorLow-20
						leftMotor.forward();
						rightMotor.forward();
					} 
					else if (average <= 20){
						System.out.println("less than 20");
						leftMotor.setSpeed(Lab3.FORWARD_SPEED); // Right turn
						rightMotor.setSpeed(100);
						leftMotor.forward();
						rightMotor.forward();
					}
				}
				//If the robot has passed the object, turn the sensor back and the robot heads to the next point
				else {
					leftMotor.stop(true);
					rightMotor.stop(true);
					sensorMotor.rotate(45);
					objectDetected = false;
					pointIndex --;
					break;
				}
			}
		}
	}

	//This method takes the minimum angle as input and turn the robot to this heading angle 
	void turnTo(double theta) {    
		leftMotor.setSpeed(Lab3.ROTATE_SPEED);
		rightMotor.setSpeed(Lab3.ROTATE_SPEED);

		leftMotor.rotate(convertAngle(Lab3.RADIUS, Lab3.TRACK, theta), true);
		rightMotor.rotate(-convertAngle(Lab3.RADIUS, Lab3.TRACK, theta), false);
	}

	//This method return true if both motors are moving
	boolean isNavigating() { 
		return leftMotor.isMoving() && rightMotor.isMoving();
	}
	/**
	 * This method converts an angle to actual wheel movement distance.
	 * @param radius the radius of the wheel
	 * @param distance the distance between the wheels
	 * @param 
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	/**
	 * This method converts a wheel distance to the equivalent agnle in terms of wheel rotation.
	 * @param radius the radius of the wheel
	 * @param width the distance between the wheels
	 * @param 
	 */
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	@Override
	public int readUSDistance() {
		return this.distance;
	}

	@Override
	public int readFilterControl() {
		return 0;
	}
}
