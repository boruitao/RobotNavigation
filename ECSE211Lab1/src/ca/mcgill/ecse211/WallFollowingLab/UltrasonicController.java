package ca.mcgill.ecse211.WallFollowingLab;

public interface UltrasonicController {

	  public void processUSData(int distance);

	  public int readUSDistance();
	  
	  public int readFilterControl();
	}
