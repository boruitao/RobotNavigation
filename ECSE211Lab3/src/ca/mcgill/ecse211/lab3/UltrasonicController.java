package ca.mcgill.ecse211.lab3;
/**
 * This class returns information from the ultrasonic sensor.
 */
public interface UltrasonicController {
	 public void processUSData(int distance);

	  public int readUSDistance();
	  
	  public int readFilterControl();
}
