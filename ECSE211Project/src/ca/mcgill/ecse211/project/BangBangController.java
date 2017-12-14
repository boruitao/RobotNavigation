package ca.mcgill.ecse211.project;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class BangBangController {
	
	private static final int FILTER_OUT = 20;
	private final int bandCenter;
	private final int bandwidth;
	private final int motorLow;
	private final int motorHigh;
	private int distance;
	private int filterControl;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private Navigation na;
	private Odometer od;

	/**
	 * The constructor initializes the values for all the parameters of the wall following and starts the motors.
	 * @param bandCenter	The distance from the wall
	 * @param bandwidth		The error margin
	 * @param motorLow		The low motor speed
	 * @param motorHigh		The high motor speed
	 * @param leftMotor		The pointer to the left motor
	 * @param rightMotor	The pointer to the right motor
	 */
	public BangBangController(Navigation na, Odometer od, int bandCenter, int bandwidth, int motorLow, int motorHigh, 
			EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
		this.bandCenter = bandCenter;
		this.bandwidth = bandwidth;
		this.motorLow = motorLow;
		this.motorHigh = motorHigh;
		this.filterControl = 0;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.leftMotor.setSpeed(motorHigh);
		this.rightMotor.setSpeed(motorHigh);
		this.leftMotor.forward();
		this.rightMotor.forward();
		this.na = na;
		this.od = od;
	}

	/**
	 * The method runs in a thread to allow the wall following as well as the flag recognition (with light sensor) to be performed in parallel.
	 */
	public void run() {
		float forwardLimit = 180;
		float backwardLimit = 2;
		float bwMotorLow = 50;
		float fwLimitSpd = 100;
		if (distance < backwardLimit){
			this.leftMotor.setSpeed(bwMotorLow);
			this.rightMotor.setSpeed(motorHigh);
			this.leftMotor.backward();
			this.rightMotor.backward();
		}else if(distance < (bandCenter - bandwidth) && distance > backwardLimit){
			this.leftMotor.setSpeed(motorHigh);
			this.rightMotor.setSpeed(motorLow);
			this.leftMotor.forward();
			this.rightMotor.forward();
		}else if(distance > (bandCenter - bandwidth) && distance < (bandCenter + bandwidth)){
			this.leftMotor.setSpeed(motorHigh);
			this.rightMotor.setSpeed(motorHigh);
			this.leftMotor.forward();
			this.rightMotor.forward();
		}else if(distance > (bandCenter + bandwidth) && distance < forwardLimit){
			this.leftMotor.setSpeed(motorLow);
			this.rightMotor.setSpeed(motorHigh);
			this.leftMotor.forward();
			this.rightMotor.forward();
		}else if(distance > forwardLimit){
			this.rightMotor.setSpeed(motorHigh);
			this.leftMotor.setSpeed(fwLimitSpd);
			this.leftMotor.forward();
			this.rightMotor.forward();

		} 
	}
	
	public void doGridTraversal(double nX, double nY, double x, double y, int length){

		double curDestX = nX;
		double curDestY = nY;
		boolean hasBlock = false;

		int counter = 2 * length;
		while(counter > 0){
			curDestX += (nX + x)/(length*2);
			curDestY += (nY + y)/(length*2);
			na.travelTo(curDestX, curDestY);
			while(na.isNavigating()){
			}
			na.rotateUltraMotor(100,false);
			System.out.println("The distance is " + distance);
			if (distance < 30){
				Sound.playNote(Sound.FLUTE, 440, 250); // sound to let us know robot sees the line
			}
			na.rotateUltraMotor(100,true);
			counter--;
		}
	}


	/**
	 * This method is called outside this class to set the value of the distance from the wall. A filter is applied.
	 * @param distance
	 */
	
	public void processUSData(int distance) {
		this.distance = distance;
	}
	
//	public void processUSData(int distance) {
//		
//		// rudimentary filter - toss out invalid samples corresponding to null
//		// signal.
//		// (n.b. this was not included in the Bang-bang controller, but easily
//		// could have).
//		//
//		if (distance >= 120 && filterControl < FILTER_OUT) {
//			// bad value, do not set the distance var, however do increment the
//			// filter value
//			filterControl++;
//		} else if (distance >= 120) {
//			// We have repeated large values, so there must actually be nothing
//			// there: leave the distance alone
//			this.distance = distance;
//		} else {
//			// distance went below 255: reset filter and leave
//			// distance alone.
//			filterControl = 0;
//			this.distance = distance;
//		}
//	}
	
	/**
	 * This method returns the distance from the wall (to be displayed on the screen for example).
	 * @return	the distance from the wall.
	 */
	public int readUSDistance() {
		return this.distance;
	}

}
