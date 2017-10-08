package ca.mcgill.ecse211.WallFollowingLab;

import lejos.hardware.motor.*;
import java.util.Arrays;

public class BangBangController implements UltrasonicController {

	 private final int bandCenter; // Desired offset from the wall (cm)
	  private final int bandwidth; // Width of dead band (cm)
	  private final int motorLow;  //Define motor speed (200 deg/sec)
	  private final int motorHigh; //Define secondary speed (100 deg/sec)
	  private int distance; //Current offset from the wall
	  private int distError; // Current offset from desired distance
	  private int[] lastFiveDistance = new int[5]; // Array of the last 5 US outputs
	  private int index = 0; //Index to keep track of the array part we are accessing.
	  
	  private int filterControl; // Counts the amount of times we have filtered the input
	  private static final int FILTER_OUT = 25; // Maximum number of times we filter large inputs


  public BangBangController(int bandCenter, int bandwidth, int motorLow, int motorHigh) {
    // Default Constructor
    this.bandCenter = bandCenter;
    this.bandwidth = bandwidth;
    this.motorLow = motorLow;
    this.motorHigh = motorHigh;
    Arrays.fill(lastFiveDistance, 30);
    WallFollowingLab.leftMotor.setSpeed(motorHigh); // Start robot moving forward
    WallFollowingLab.rightMotor.setSpeed(motorHigh);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
    filterControl = 0;
    distError = 0;
  }

  @Override
  public void processUSData(int distance) {
	  // Because the US data tends to be inconsistent we take the average of 
	  // 5 values from the sensor and set it equal to distance.
	  // To prevent high sensor values from skewing the average we set values greater than 100 to 100.
	  
	    distance = Math.min(distance, 100);
	    this.index++;
	    if(this.index == 5) {
	    	this.index = 0;
	    }
	    
	    lastFiveDistance[index] = distance;
	    
	    int average = 0;
	    for (int i=0; i<lastFiveDistance.length; i++) {
	    	average+=lastFiveDistance[i];
	    }
	    average = average / 5;
	    
	    this.distance = average;
	    // Basic filter used to ignore bad inputs with high values for a short amount of time
	   
	    distError = this.distance - this.bandCenter;
    if ((distance >= 70 || distance <=0) && filterControl < FILTER_OUT) {
        // bad value, do not set the distance var, however do increment the
        // filter value
        filterControl++;
        return;
      } else if (distance >= 70) {
        // We have repeated large values, so there must actually be nothing
        // there: leave the distance alone
        this.distance = 70;
      } else if (distance <= 70){
        // distance went below 255: reset filter and leave
        // distance alone.
        filterControl = 0;
        this.distance = distance;
      }
    
    if (Math.abs(distError) <= this.bandwidth) { 
    	// Within limits, same speed 
    	WallFollowingLab.leftMotor.setSpeed(motorHigh); // Start moving forward 
    	WallFollowingLab.rightMotor.setSpeed(motorHigh);
    	WallFollowingLab.leftMotor.forward();
    	WallFollowingLab.rightMotor.forward(); 
    }
    
    else if (distError > 0) {  //turns left
    	WallFollowingLab.leftMotor.setSpeed(motorLow-20);
    	WallFollowingLab.rightMotor.setSpeed(motorHigh+30); 
    	WallFollowingLab.leftMotor.forward();
    	WallFollowingLab.rightMotor.forward(); 
    }
    
    else if (distError < 0) { //turns right
       	WallFollowingLab.leftMotor.setSpeed(motorHigh+37); 
    	WallFollowingLab.rightMotor.setSpeed(motorLow-55);
    	WallFollowingLab.leftMotor.forward();
    	WallFollowingLab.rightMotor.forward(); 
    }
    
    
  }

  @Override
  public int readUSDistance() {
    return this.distance;
  }
  
  public int readFilterControl(){
	  return this.filterControl;
  }
}