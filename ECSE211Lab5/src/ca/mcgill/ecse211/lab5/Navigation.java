package ca.mcgill.ecse211.lab5;

/** This class takes care of the robot navigation. It controls the movement by
 * 	implementing movement, turning, distance & angle methods.
 * 	@author Antonios Valkanas, Borui Tao
 * 	@version 1.0
 * 
 */

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Navigation{
	//variables only set once
	private Odometer odometer; 
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private EV3LargeRegulatedMotor ziplineMotor;

	//changing variables 
	private boolean isNavigating;
	private boolean isTurning;
	double[] position = new double[3];
	private double nowX;
	private double nowY;
	private double nowTheta;
	private double thetaObj;

	//constructor
	public Navigation(Odometer odometer,EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, EV3LargeRegulatedMotor ziplineMotor) {
		this.odometer = odometer;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.ziplineMotor = ziplineMotor;
	} 

	/**
	 * This method makes the robot to travel to a specific location (x,y)
	 * First it calculates the heading angle that the robot must face
	 * Then it gets the minimum turning angle which the robot must turn to
	 * Afterwards it calls the method makeCorrectedTurn() to turn to that angle.
	 * Finally it calculates the traveling distance and travel to that distance.
	 * 
	 * @param  x   The x-coordinate of the destination point 
	 * @param  y   The y-coordinate of the destination point 
	 */
	
	void travelTo(double x, double y){

		isNavigating=true;

		nowX = odometer.getX();
		nowY = odometer.getY();
		//calculate the angle we need to turn to
		double theta1 = Math.atan((x-nowX)/(y-nowY))*360.0/(2*Math.PI);
		if(x-nowX<0) theta1= 180.0 + theta1;
		//turn to the proper angle
		makeMinimumTurn(theta1);
		
		double travellingDis = Math.sqrt(Math.pow(x-nowX, 2) + Math.pow(y-nowY, 2));

		//drive forward
		leftMotor.setSpeed(ZiplineLab.FORWARD_SPEED);
		rightMotor.setSpeed(ZiplineLab.FORWARD_SPEED);
		leftMotor.rotate(convertDistance(ZiplineLab.RADIUS, travellingDis), true);
		rightMotor.rotate(convertDistance(ZiplineLab.RADIUS, travellingDis), true);

		//keep calling turnto and checking the distance
		isNavigating=false;
	}
	
	void doZipline(double distance){
		//drive forward
		leftMotor.setSpeed(ZiplineLab.FORWARD_SPEED);
		rightMotor.setSpeed(ZiplineLab.FORWARD_SPEED);
		leftMotor.rotate(convertDistance(ZiplineLab.RADIUS, distance/2), true);
		rightMotor.rotate(convertDistance(ZiplineLab.RADIUS, distance/2), true);
		
		ziplineMotor.setSpeed(2*ZiplineLab.FORWARD_SPEED);
		ziplineMotor.rotate(-convertDistance(ZiplineLab.RADIUS, 6*ZiplineLab.ZIPLENGTH), false);
		
		leftMotor.setSpeed(ZiplineLab.FORWARD_SPEED);
		rightMotor.setSpeed(ZiplineLab.FORWARD_SPEED);
		leftMotor.rotate(convertDistance(ZiplineLab.RADIUS, 5), true);
		rightMotor.rotate(convertDistance(ZiplineLab.RADIUS, 5), true);
	}
	
	/**
	 * This method makes the robot turn theta degrees.
	 * @param theta 		angle of rotation
	 */
	void makeTurn(double theta) {
	    leftMotor.setSpeed(ZiplineLab.ROTATE_SPEED);
	    rightMotor.setSpeed(ZiplineLab.ROTATE_SPEED);

	    // Rotate to new angle
	    leftMotor.rotate(convertAngle(ZiplineLab.RADIUS, ZiplineLab.TRACK, theta), true);
	    rightMotor.rotate(-convertAngle(ZiplineLab.RADIUS, ZiplineLab.TRACK, theta), true);
	  }


	/**
	 * This method makes the robot turn using the minimal angle. 
	 * The point is to avoid turns greater than 180 degrees by changing direction.
	 * @param theta 		angle of rotation
	 */
	void makeMinimumTurn(double theta){
		 theta = ((theta % 360) + 360) % 360;
		 if (theta >= 180) theta-=360;
		 leftMotor.setSpeed(ZiplineLab.ROTATE_SPEED);
		 rightMotor.setSpeed(ZiplineLab.ROTATE_SPEED);

		 // Rotate to new angle
		 leftMotor.rotate(convertAngle(ZiplineLab.RADIUS, ZiplineLab.TRACK, theta), true);
		 rightMotor.rotate(-convertAngle(ZiplineLab.RADIUS, ZiplineLab.TRACK, theta), false);
	}
	
	/**
	 * This method return true if both motors are moving or the robot is traveling or is turning
	 * 
	 * @return 			whether the motors are moving
	 */
	boolean isNavigating(){
		//returns true if the robot is either Navigating or Turning
		return isNavigating || isTurning || leftMotor.isMoving() && rightMotor.isMoving();
	}
	
	/**
	 * This method stops the robot from moving by setting the speed of motors to zero
	 */

	public void motorStop(){
		leftMotor.setSpeed(0);
		rightMotor.setSpeed(0);
	}
	
	/**
	 * This method get current odometer readings and populates the data to the class 
	 * fields nowX, nowY, nowTheta. 
	 * 
	 * @param odometer		the odometer object
	 */
	
	void getPosition(Odometer odometer){
		odometer.getPosition(position, new boolean[] {true, true, true});
		nowX = position[0];
		nowY = position[1];
		nowTheta = position[2];				 
	}

	/**
	 * This method converts an angle to actual wheel movement distance.
	 * @param radius 		the radius of the wheel
	 * @param distance 		the distance between the wheels. 
	 * @return 			    the distance that has been calculated 
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	
	/**
	 * This method converts a wheel distance to the equivalent agnle in terms of wheel rotation.
	 * @param radius 		the radius of the wheel
	 * @param width 		the distance between the wheels
	 * @param angle			the angle we wish to convert 
	 * @return 				conversion from angle to wheel rotation distance
	 */
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	
	
	//Raphael's methods
	public void raphTravelTo(double x, double y){
		//get the current values for x, y and theta
		nowX = odometer.getX();
		nowY = odometer.getY();
		nowTheta = odometer.getTheta();
		double alpha;
		//get the distance in cm for x and y
		//x *= 30.48;
		//y *= 30.48;
		alpha = Math.atan(Math.abs(x-nowX)/Math.abs(y-nowY));//get the angle alpha 
		alpha = (180*alpha)/(Math.PI);					//convert in degrees
		//in this if conditions, it will get the real angle it has to go (from the "origin", the y axis)
		if(y > nowY){
			if(x < nowX){
				thetaObj = 360 - alpha;
			}else if(x > nowX){
				thetaObj = alpha;
			}else{
				thetaObj = 0;
			}
		}else if(y < nowY){
			if(x < nowX){
				thetaObj = 180 + alpha;
			}else if(x > nowX){
				thetaObj = 180 - alpha;
			}else{
				thetaObj = 180;
			}
		}else if(x < nowX){ //here y = yCur
			thetaObj = 270;
		}else{ //here y = yCur and x > xCur
			thetaObj = 90;
		}
		raphTurnTo(thetaObj);//turn to this angle
	    //calculate the distance the robot has to cover
	    double distance = Math.sqrt(Math.pow(y-nowY,2) + Math.pow(x-nowX,2));
	    ///rotate for this distance in cm'
		leftMotor.setSpeed(ZiplineLab.FORWARD_SPEED);
		rightMotor.setSpeed(ZiplineLab.FORWARD_SPEED);
		
		leftMotor.rotate(convertDistance(ZiplineLab.RADIUS, distance), true);
	    rightMotor.rotate(convertDistance(ZiplineLab.RADIUS, distance), false);
	}
	
	public void raphTurnTo(double theta){
		nowTheta = odometer.getTheta();
		//get the displacement (difference between the current angle and where you want to go.
		double displacement = Math.abs(nowTheta - theta);
		//turn accordingly, making sure it is the minimal angles
		if (theta < nowTheta){
			if(displacement < 180){
				leftMotor.rotate(-convertAngle(ZiplineLab.RADIUS, ZiplineLab.TRACK, displacement), true);
				rightMotor.rotate(convertAngle(ZiplineLab.RADIUS, ZiplineLab.TRACK, displacement), false);
			}else{
				displacement = 360 - displacement;
				leftMotor.rotate(convertAngle(ZiplineLab.RADIUS, ZiplineLab.TRACK, displacement), true);
				rightMotor.rotate(-convertAngle(ZiplineLab.RADIUS, ZiplineLab.TRACK, displacement), false);
			}
		}else{
			if(displacement < 180){
				leftMotor.rotate(convertAngle(ZiplineLab.RADIUS, ZiplineLab.TRACK, displacement), true);
				rightMotor.rotate(-convertAngle(ZiplineLab.RADIUS, ZiplineLab.TRACK, displacement), false);
			}else{
				displacement = 360 - displacement;
				leftMotor.rotate(-convertAngle(ZiplineLab.RADIUS, ZiplineLab.TRACK, displacement), true);
				rightMotor.rotate(convertAngle(ZiplineLab.RADIUS, ZiplineLab.TRACK, displacement), false);
			}
		}
	}
	
}
