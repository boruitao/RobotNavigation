package ca.mcgill.ecse211.lab4;

/**
 * @author Antonios Valkanas, Borui Tao
 * @version 1.0
 * 
 */

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Navigation{
	//variables only set once
	private Odometer odometer; 
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private double wheelRadius;
	private double width; 

	//changing variables 
	private boolean isNavigating;
	private boolean isTurning;
	double[] position = new double[3];
	private double nowX;
	private double nowY;
	private double nowTheta;

	//constructor
	public Navigation(Odometer odometer,EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, 
			double wheelRadius, double width) {

		this.odometer = odometer;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.wheelRadius = wheelRadius;
		this.width = width;


	} 

	void travelTo(double x, double y){

		isNavigating=true;

		nowX = odometer.getX();
		nowY = odometer.getY();
		//calculate the angle we need to turn to
		double theta1 = Math.atan((y-nowY)/(x-nowX))*360.0/(2*Math.PI);
		if(x-nowX<0) theta1= 180.0 + theta1;
		//turn to the proper angle
		makeCorrectedTurn(theta1);
		
		System.out.println("X is " + x + " and " + "Y is " + y);
		System.out.println("X now is " + nowX + " and " + "Y now is " + nowY);
		
		double travellingDis = Math.sqrt(Math.pow(x-nowX, 2) + Math.pow(y-nowY, 2));
		System.out.println("the travDis is" + travellingDis);
		//drive forward
		leftMotor.setSpeed(LocalizationLab.FORWARD_SPEED);
		rightMotor.setSpeed(LocalizationLab.FORWARD_SPEED);
		leftMotor.rotate(convertDistance(LocalizationLab.RADIUS, travellingDis), true);
		rightMotor.rotate(convertDistance(LocalizationLab.RADIUS, travellingDis), true);

		//keep calling turnto and checking the distance
		isNavigating=false;
	}

	void makeTurn(double theta) {
	    leftMotor.setSpeed(LocalizationLab.ROTATE_SPEED);
	    rightMotor.setSpeed(LocalizationLab.ROTATE_SPEED);

	    // Rotate to new angle
	    leftMotor.rotate(convertAngle(LocalizationLab.RADIUS, LocalizationLab.TRACK, theta), true);
	    rightMotor.rotate(-convertAngle(LocalizationLab.RADIUS, LocalizationLab.TRACK, theta), true);
	  }


	void makeCorrectedTurn(double theta){
		 theta = ((theta % 360) + 360) % 360;
		 if (theta >= 180) theta-=360;
		 leftMotor.setSpeed(LocalizationLab.ROTATE_SPEED);
		 rightMotor.setSpeed(LocalizationLab.ROTATE_SPEED);

		 // Rotate to new angle
		 leftMotor.rotate(convertAngle(LocalizationLab.RADIUS, LocalizationLab.TRACK, theta), true);
		 rightMotor.rotate(-convertAngle(LocalizationLab.RADIUS, LocalizationLab.TRACK, theta), false);
	}
	void turnTo(double theta){

		isTurning =  true;
		//convert to polar coordinate
		nowTheta=(360.0-nowTheta)+90.0;   
		if ( nowTheta>360.0) nowTheta= nowTheta-360.0;
		//calculate the angle we need to turn
		double turningTheta=theta-nowTheta;
		//make sure it is the minimal angle
		if (turningTheta>180) turningTheta=turningTheta-360.0;
		else if (turningTheta<-180) turningTheta=turningTheta+360.0;

		//make a turn
		leftMotor.setSpeed(LocalizationLab.ROTATE_SPEED);
		rightMotor.setSpeed(LocalizationLab.ROTATE_SPEED);
		leftMotor.rotate(-convertAngle(wheelRadius, width, turningTheta), true);
		rightMotor.rotate(convertAngle(wheelRadius, width, turningTheta), false);
		isTurning = false;	
	}

	boolean isNavigating(){
		//returns true if the robot is either Navigating or Turning
		return isNavigating || isTurning || leftMotor.isMoving() && rightMotor.isMoving();
	}

	public void motorStop(){
		leftMotor.setSpeed(0);
		rightMotor.setSpeed(0);
	}
	
	void getPosition(Odometer odometer){
		odometer.getPosition(position, new boolean[] {true, true, true});
		nowX = position[0];
		nowY = position[1];
		nowTheta = position[2];				 
	}

	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
}
