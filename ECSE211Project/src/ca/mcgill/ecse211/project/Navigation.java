package ca.mcgill.ecse211.project;


/** 
 * Navigation class - test javadoc generation
 * 	@author Raphael Di Piazza
 * 	@version 1.0
 * 
 */

import lejos.hardware.motor.EV3LargeRegulatedMotor;


/**
 * This class takes care of the robot navigation. It controls the movement by
 * implementing movement, turning, distance & angle methods.
 */
public class Navigation{
	//variables only set once
	/**
	 * The pointer to the odometer class.
	 */
	private Odometer odometer;
	/**
	 * The pointer to the left motor.
	 */
	private EV3LargeRegulatedMotor leftMotor;
	/**
	 * The pointer to the right motor.
	 */
	private EV3LargeRegulatedMotor rightMotor;
	/**
	 * The pointer to the motor connected to the ultrasonic sensor.
	 */
	private EV3LargeRegulatedMotor ulMotor;
	/**
	 * The pointer to the motor used for the zipline traversal.
	 */
	private EV3LargeRegulatedMotor ziplineMotor;

	//changing variables 
	/**
	 * The boolean to know when the robot is navigating.
	 */
	private boolean isNavigating;
	/**
	 * The boolean to know when the robot is turning on itself.
	 */
	private boolean isTurning;
	/**
	 * The array for the robot's position (index 0 = x, index 1 = y, index 2 = theta).
	 */
	double[] position = new double[3];
	/**
	 * The value of the current x.
	 */
	private double nowX;
	/**
	 * The value of the current y.
	 */
	private double nowY;
	/**
	 * The value of the current theta.
	 */
	private double nowTheta;
	/**
	 * The objective theta: the target theta where it wants to go.
	 */
	private double thetaObj;
	
	/**
	 * The corrected X coordinate (the exact location where the robot is).
	 */
	public double currentX;
	
	/**
	 * The corrected Y coordinate (the exact location where the robot is).
	 */
	public double currentY;
	
	/**
	 * The constructor for the Navigation that initializes the 3 motors as well as the odometer.
	 * @param odometer		pointer to the Odometer
	 * @param leftMotor		pointer to the left motor
	 * @param rightMotor	pointer to the right motor
	 * @param ziplineMotor	pointer to the zipline motor
	 */
	public Navigation(Odometer odometer,EV3LargeRegulatedMotor leftMotor, 
			EV3LargeRegulatedMotor rightMotor, EV3LargeRegulatedMotor ziplineMotor, EV3LargeRegulatedMotor ulMotor) {
		this.odometer = odometer;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.ulMotor = ulMotor;
		this.ziplineMotor = ziplineMotor;
	} 
	
	/**
	 * This method performs the Zipline traversal. by making the robot gowing towards the start point.
	 * It then starts the zipline motor to traverse the river.
	 * 
	 * @param  distance		the total length of the zipline.
	 */
	void doZipline(double distance){
		//drive forward
		leftMotor.setSpeed(CaptureFlag.FORWARD_SPEED + 120);
		rightMotor.setSpeed(CaptureFlag.FORWARD_SPEED + 120);
		leftMotor.rotate(convertDistance(CaptureFlag.RADIUS, 0.71*distance), true);
		rightMotor.rotate(convertDistance(CaptureFlag.RADIUS, 0.71*distance), true);
		
		ziplineMotor.setSpeed(2*CaptureFlag.FORWARD_SPEED);
		ziplineMotor.rotate(-convertDistance(CaptureFlag.RADIUS, 3.5*CaptureFlag.ZIPLENGTH), false);
		
		/*leftMotor.setSpeed(CaptureFlag.FORWARD_SPEED);
		rightMotor.setSpeed(CaptureFlag.FORWARD_SPEED);
		leftMotor.rotate(convertDistance(CaptureFlag.RADIUS, 5), true);
		rightMotor.rotate(convertDistance(CaptureFlag.RADIUS, 5), true);*/
	}
	
	/**
	 * This methods performs the zipline traversal by firstly going to the input target coordinates (start of the zipline).
	 * @param x				the x coordinate for the start of the zipline.
	 * @param y				the y coordinate for the start of the zipline.
	 * @param zipdistance	the distance of the zipline.
	 */
	public void doZipline(double x, double y, double zipdistance, double travelDis){
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
		turnTo(thetaObj);//turn to this angle
		//calculate the distance the robot has to cover
		double distance = Math.sqrt(Math.pow(y-nowY,2) + Math.pow(x-nowX,2));

		///rotate for this distance in cm'
		//drive forward
		leftMotor.setSpeed(CaptureFlag.ZIPLINE_SPEED);
		rightMotor.setSpeed(CaptureFlag.ZIPLINE_SPEED);
		leftMotor.rotate(convertDistance(CaptureFlag.RADIUS, 0.77*zipdistance), true);   //0.77*distance
		rightMotor.rotate(convertDistance(CaptureFlag.RADIUS, 0.77*zipdistance), true);
		
		ziplineMotor.setSpeed(3*CaptureFlag.FORWARD_SPEED);
		ziplineMotor.rotate(-convertDistance(CaptureFlag.RADIUS, 4.5*CaptureFlag.ZIPLENGTH), false); //3.5
		
		leftMotor.rotate(convertDistance(CaptureFlag.RADIUS, travelDis), true);   //0.77*distance
		rightMotor.rotate(convertDistance(CaptureFlag.RADIUS, travelDis), false);

		/*leftMotor.setSpeed(CaptureFlag.FORWARD_SPEED);
		rightMotor.setSpeed(CaptureFlag.FORWARD_SPEED);
		leftMotor.rotate(convertDistance(CaptureFlag.RADIUS, 5), true);
		rightMotor.rotate(convertDistance(CaptureFlag.RADIUS, 5), true);*/
	}
	
	/**
	 * This method permits the robot to  performs a security turn before a midpoint localization to make sure it gets the first line.
	 */
	void securityTurn(){
		leftMotor.setSpeed(CaptureFlag.ROTATE_SPEED);
		rightMotor.setSpeed(CaptureFlag.ROTATE_SPEED);
		leftMotor.rotate(-100, true);
		rightMotor.rotate(100, false);
	}
	
	/**
	 * This method is to know if the robot is traversing the zipline.
	 * @return true if the robot is currently on the zipline and false otherwise.
	 */
	boolean onZipline(){
		//returns true if the robot is either Navigating or Turning
		return ziplineMotor.isMoving();
	}
	
	/**
	 * This method makes the robot to travel to a specific location (x,y)
	 * From the current direction and the coordinates to go to, it calculates the angle to go to,
	 * then calls the turnTo method to face the target point and then travels to this point.
	 * 
	 * @param  x	The x-coordinate of the destination point.
	 * @param  y	The y-coordinate of the destination point.
	 * @return		Whether the robot arrived to its target destination or stopped at midpoint to allow localization.
	 */
	public boolean travelTo(double x, double y){
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
		turnTo(thetaObj);//turn to this angle
	    //calculate the distance the robot has to cover
	    double distance = Math.sqrt(Math.pow(y-nowY,2) + Math.pow(x-nowX,2));
	    ///rotate for this distance in cm'
		leftMotor.setSpeed(CaptureFlag.FORWARD_SPEED);
		rightMotor.setSpeed(CaptureFlag.FORWARD_SPEED);
		
//		leftMotor.rotate(convertDistance(CaptureFlag.RADIUS, distance), true);
//	    rightMotor.rotate(convertDistance(CaptureFlag.RADIUS, distance), false);
	    
	  //do lightLocalization if distance > 3 tiles
	  	if(distance > (3*CaptureFlag.SQUARE_LENGTH+5)){
	  			leftMotor.rotate(convertDistance(CaptureFlag.RADIUS,3*CaptureFlag.SQUARE_LENGTH), true);
	  			rightMotor.rotate(convertDistance(CaptureFlag.RADIUS,3*CaptureFlag.SQUARE_LENGTH), false);
	  			setCurrentCoordinates();
	  			return false;
	  		}else{
	  			leftMotor.rotate(convertDistance(CaptureFlag.RADIUS,distance), true);
	  			rightMotor.rotate(convertDistance(CaptureFlag.RADIUS,distance), false);
	  			setCurrentCoordinates();
	  			return true;
	  		}
	}
	
	/**
	 * This method is used before the first localization to go to the "origin" point (its not always (0;0) because
	 * it depends on the starting corner.
	 * @param x		The x-coordinate of the destination point.
	 * @param y		The y-coordinate of the destination point.
	 */
	void goToOrigin(double x, double y){

		isNavigating=true;

		nowX = odometer.getX();
		nowY = odometer.getY();
		//calculate the angle we need to turn to
		double theta1 = Math.atan((x-nowX)/(y-nowY))*360.0/(2*Math.PI);
		if(x-nowX<0) theta1= 180.0 + theta1;
		//turn to the proper angle
		makeTurn(theta1, true, false);
		
		double travellingDis = Math.sqrt(Math.pow(x-nowX, 2) + Math.pow(y-nowY, 2));

		//drive forward
		leftMotor.setSpeed(CaptureFlag.FORWARD_SPEED);
		rightMotor.setSpeed(CaptureFlag.FORWARD_SPEED);
		leftMotor.rotate(convertDistance(CaptureFlag.RADIUS, travellingDis), true);
		rightMotor.rotate(convertDistance(CaptureFlag.RADIUS, travellingDis), true);

		//keep calling turnto and checking the distance
		isNavigating=false;
	}
	
	/**
	 * This method makes the robot face the wanted angle direction from the current direction.
	 * @param theta		the target direction.
	 */
	public void turnTo(double theta){
		nowTheta = odometer.getTheta();
		//get the displacement (difference between the current angle and where you want to go.
		double displacement = Math.abs(nowTheta - theta);
		//turn accordingly, making sure it is the minimal angles
		if (theta < nowTheta){
			if(displacement < 180){
				leftMotor.rotate(-convertAngle(CaptureFlag.RADIUS, CaptureFlag.TRACK, displacement), true);
				rightMotor.rotate(convertAngle(CaptureFlag.RADIUS, CaptureFlag.TRACK, displacement), false);
			}else{
				displacement = 360 - displacement;
				leftMotor.rotate(convertAngle(CaptureFlag.RADIUS, CaptureFlag.TRACK, displacement), true);
				rightMotor.rotate(-convertAngle(CaptureFlag.RADIUS, CaptureFlag.TRACK, displacement), false);
			}
		}else{
			if(displacement < 180){
				leftMotor.rotate(convertAngle(CaptureFlag.RADIUS, CaptureFlag.TRACK, displacement), true);
				rightMotor.rotate(-convertAngle(CaptureFlag.RADIUS, CaptureFlag.TRACK, displacement), false);
			}else{
				displacement = 360 - displacement;
				leftMotor.rotate(-convertAngle(CaptureFlag.RADIUS, CaptureFlag.TRACK, displacement), true);
				rightMotor.rotate(convertAngle(CaptureFlag.RADIUS, CaptureFlag.TRACK, displacement), false);
			}
		}
	}
	
	/**
	 * This method makes the robot turn theta degrees.
	 * @param theta 		angle of rotation
	 */
	void makeTurn(double theta, boolean isMin, boolean immediate) {
		if (isMin){
			 theta = ((theta % 360) + 360) % 360;
			 if (theta >= 180) theta-=360;
		}
	    leftMotor.setSpeed(CaptureFlag.ROTATE_SPEED);
	    rightMotor.setSpeed(CaptureFlag.ROTATE_SPEED);

	    // Rotate to new angle
	    leftMotor.rotate(convertAngle(CaptureFlag.RADIUS, CaptureFlag.TRACK, theta), true);
	    rightMotor.rotate(-convertAngle(CaptureFlag.RADIUS, CaptureFlag.TRACK, theta), true && immediate);
	  }
	
	/**
	 * This method rotates the motor connected to the ultrasonic sensor to perform flag detection.
	 * @param back	boolean to know if we turn on the right or on the left.
	 */
	void rotateUltraMotor(int deg,boolean back) {
		ulMotor.setSpeed(CaptureFlag.ROTATE_SPEED);

	    // Rotate to new angle
		if (!back) ulMotor.rotate(-deg, false);
		else ulMotor.rotate(deg,false);
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
	 * This method makes the motors move forward.
	 */
	public void motorMoveForward(){
		leftMotor.setSpeed(CaptureFlag.ROTATE_SPEED);
		rightMotor.setSpeed(CaptureFlag.ROTATE_SPEED);
		
		leftMotor.forward();
		rightMotor.forward();
	}
	
	/**
	 * This method makes the motors move backwards for a fixed distance.
	 * 
	 * @param distance		the distance for the motor to move.
	 */
	public void motorMoveBackward(double distance){
		leftMotor.setSpeed(CaptureFlag.ROTATE_SPEED+10);
		rightMotor.setSpeed(CaptureFlag.ROTATE_SPEED+10);
			
		leftMotor.rotate(-convertDistance(CaptureFlag.RADIUS, distance), true);
		rightMotor.rotate(-convertDistance(CaptureFlag.RADIUS, distance), false);

	}
	
	/**
	 * This method stops the robot from moving by setting the speed of motors to zero.
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
	
	/**
	 * This method is used after the midpoints localization to set the x and y to their exact values.
	 */
	private void setCurrentCoordinates(){
		double shiftX = odometer.getX() % CaptureFlag.SQUARE_LENGTH;
		if(shiftX > (CaptureFlag.SQUARE_LENGTH / 2)){
			this.currentX = odometer.getX() + (CaptureFlag.SQUARE_LENGTH - shiftX);
		}else{
			this.currentX = odometer.getX() - shiftX;
		}
		double shiftY = odometer.getY() % CaptureFlag.SQUARE_LENGTH;
		if(shiftY > (CaptureFlag.SQUARE_LENGTH / 2)){
			this.currentY = odometer.getY() + (CaptureFlag.SQUARE_LENGTH - shiftY);
		}else{
			this.currentY = odometer.getY() - shiftY;
		}
	}
	
}
