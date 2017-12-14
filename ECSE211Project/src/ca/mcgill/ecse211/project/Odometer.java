package ca.mcgill.ecse211.project;

/**
 * Odometer class
 * @author Antonios Valkanas, Borui Tao
 * @version 1.0
 * 
 */

import lejos.hardware.motor.EV3LargeRegulatedMotor;


/**
 * The odometer takes care of updating the values for X, Y and Theta based on the wheel
 * motors tacho count.
 */
public class Odometer extends Thread {
	  // robot position
	  private double x;
	  private double y;
	  private double theta;
	  private int leftMotorTachoCount;
	  private int rightMotorTachoCount;
	  private EV3LargeRegulatedMotor leftMotor;
	  private EV3LargeRegulatedMotor rightMotor;


	  private static final long ODOMETER_PERIOD = 25; /*odometer update period, in ms*/

	  private Object lock; /*lock object for mutual exclusion*/

	  /**
	   * The constructor for the odometer which sets the two motors (left and right) and resets the values fr x,y,theta and tacho counts.
	   * @param	leftMotor	pointer to the left motor
	   * @param rightMotor	pointer to the right motor
	   */
	  public Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
	    this.leftMotor = leftMotor;
	    this.rightMotor = rightMotor;
	    this.x = 0.0;
	    this.y = 0.0;
	    this.theta = 0.0;
	    this.leftMotorTachoCount = 0;
	    this.rightMotorTachoCount = 0;
	    lock = new Object();
	  }

	  /**
	   * This is the run method (required for Thread)
	   */
	  public void run() {
	    long updateStart, updateEnd;

	    while (true) {
	      updateStart = System.currentTimeMillis();
	      //Get current tacho counts
	      int nowLeftMotorTachoCount = leftMotor.getTachoCount();
	      int nowRightMotorTachoCount = rightMotor.getTachoCount();

	      //Get wheel displacements
	      double distanceLeft = Math.PI * CaptureFlag.RADIUS * (nowLeftMotorTachoCount - getLeftMotorTachoCount()) / 180;
	      double distanceRight = Math.PI * CaptureFlag.RADIUS * (nowRightMotorTachoCount - getRightMotorTachoCount()) / 180;

	      //Save new tacho counts
	      setLeftMotorTachoCount(nowLeftMotorTachoCount);
	      setRightMotorTachoCount(nowRightMotorTachoCount);

	      double deltaD = 0.5 * (distanceLeft + distanceRight); //Vehicle displacement
	      double deltaT = (distanceLeft - distanceRight) / CaptureFlag.TRACK; //Change in heading
	      
	      double deltaX = deltaD * Math.sin((getTheta() * Math.PI) / 180); //X component (convert theta to radians)
	      double deltaY = deltaD * Math.cos((getTheta() * Math.PI) / 180); //Y component (convert theta to radians)
	      
	      deltaT = (deltaT * 180)/Math.PI; //Convert from radians to degrees

	      synchronized (lock) {
	        /**
	         * Don't use the variables x, y, or theta anywhere but here! Only update the values of x, y,
	         * and theta in this block. Do not perform complex math
	         * 
	         */
	          setX(getX() + deltaX); //Update estimate for x
	          setY(getY() + deltaY); //Update estimate for y
	          setTheta((getTheta() + deltaT) % 360); // Update estimate for theta and loop if greater than 360 deg
	          if(getTheta() < 0)
	        	  setTheta(getTheta() + 360); // no negative theta
	      }

	      // this ensures that the odometer only runs once every period
	      updateEnd = System.currentTimeMillis();
	      if (updateEnd - updateStart < ODOMETER_PERIOD) {
	        try {
	          Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
	        } catch (InterruptedException e) {
	          // there is nothing to be done here because it is not
	          // expected that the odometer will be interrupted by
	          // another thread
	        }
	      }
	    }
	  }

	  /**
	   * Gets the current position information of the robot.
	   * @param position 		the current position of the robot
	   * @param update 			set to true at postion 0 to get x, 1 to get y, 2 to get theta [x, y, z] 
	   */
	  public void getPosition(double[] position, boolean[] update) {
	    // ensure that the values don't change while the odometer is running
	    synchronized (lock) {
	      if (update[0])
	        position[0] = x;
	      if (update[1])
	        position[1] = y;
	      if (update[2])
	        position[2] = theta;
	    }
	  }

	  /**
	   * This method returns x
	   * @return 		 x value
	   */
	  public double getX() {
	    double result;

	    synchronized (lock) {
	      result = x;
	    }

	    return result;
	  }

	  /**
	   * This method returns y
	   * @return 			y value
	   */
	  public double getY() {
	    double result;

	    synchronized (lock) {
	      result = y;
	    }

	    return result;
	  }

	/**
	 * This method returns theta
	 * @return 			theta value
	 */
	  public double getTheta() {
	    double result;

	    synchronized (lock) {
	      result = theta;
	    }

	    return result;
	  }

	 /**
	  * This method is used to update the position of the odometer.
	  * @param position 		the current position vector
	  * @param update 			the new position vector
	  */
	  public void setPosition(double[] position, boolean[] update) {
	    // ensure that the values don't change while the odometer is running
	    synchronized (lock) {
	      if (update[0])
	        x = position[0];
	      if (update[1])
	        y = position[1];
	      if (update[2])
	        theta = position[2];
	    }
	  }
	  /**
	   * This method allows us to set x
	   * @param x 		new x value 
	   */  
	  public void setX(double x) {
	    synchronized (lock) {
	      this.x = x;
	    }
	  }
	  /**
	   * This method allows us to set y
	   * @param y 		new y value 
	   */  
	  public void setY(double y) {
	    synchronized (lock) {
	      this.y = y;
	    }
	  }
	 /**
	  * This method allows us to set Theta
	  * @param theta 		new angle value 
	  */
	  public void setTheta(double theta) {
	    synchronized (lock) {
	      this.theta = theta;
	    }
	  }

	  /**
	   * This method returns the left motor tacho count.
	   * @return 		the leftMotorTachoCount
	   */
	  public int getLeftMotorTachoCount() {
	    return leftMotorTachoCount;
	  }

	  /**
	   * This method sets the left motor tacho count.
	   * @param leftMotorTachoCount			 the leftMotorTachoCount to set
	   */
	  public void setLeftMotorTachoCount(int leftMotorTachoCount) {
	    synchronized (lock) {
	      this.leftMotorTachoCount = leftMotorTachoCount;
	    }
	  }

	  /**
	   * This method returns the right motor tacho count.
	   * @return 		the rightMotorTachoCount
	   */
	  public int getRightMotorTachoCount() {
	    return rightMotorTachoCount;
	  }

	  /**
	   * This method sets the right motor tacho count.
	   * @param rightMotorTachoCount 		the rightMotorTachoCount to set
	   */
	  public void setRightMotorTachoCount(int rightMotorTachoCount) {
	    synchronized (lock) {
	      this.rightMotorTachoCount = rightMotorTachoCount;
	    }
	  }
}