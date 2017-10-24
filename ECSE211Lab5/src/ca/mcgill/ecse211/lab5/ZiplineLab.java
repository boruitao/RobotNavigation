package ca.mcgill.ecse211.lab5;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class ZiplineLab {

	private static final EV3LargeRegulatedMotor leftMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));

	private static final EV3LargeRegulatedMotor rightMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	
	private static final EV3LargeRegulatedMotor ziplineMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
	
	private static final Port usPort = LocalEV3.get().getPort("S1");

	private static UltrasonicLocalizer ultraLoc;
	private static LightLocalizer lightLoc;
	
	//static values
	public static final int FORWARD_SPEED = 140;		  // the speed at which the robot is traveling straight
	public static final int ROTATE_SPEED = 70;		  // the speed at which the robot is rotating
	public static final double RADIUS = 2.12; 		  // radius of the wheel
	public static final double TRACK = 11.72; 		  // distance between wheels
	public static final int DISTANCE_THRESHOLD = 30;  // the distance from which the ultrasonic sensor detects the wall
	public static final int NOISE_MARGIN = 1;	  	  // the noise margin
	public static final double SQUARE_LENGTH = 30.48; // the length of the square tile
	public static final double ZIPLENGTH =70;
	public static final int[][] CORNERS = new int[][] {
		{1,1,0},
		{7,1,270},
		{7,7,180},
		{1,7,90}
	}; // the coordinates of corners
	public static int cornerCounter;
	
	public static void main(String[] args) {
		int buttonChoice;
		cornerCounter = 0;
		int bestCorner = 0;
		int xoCounter = 0;
		int yoCounter = 0;
		int xcCounter = 0;
		int ycCounter = 0;
	
		double curX;
		double curY;
		double curTheta;
	
		// clear the display
		@SuppressWarnings("resource")							    // Because we don't bother to close this resource
		SensorModes ultrasonicSensor = new EV3UltrasonicSensor(usPort);		// usSensor is the instance
		SampleProvider usDistance = ultrasonicSensor.getMode("Distance");	// usDistance provides samples from this instance
		float[] usData = new float[1];		// usData is the buffer in which data are returned
		UltrasonicPoller usPoller = null;									// the selected controller on each cycle


		final TextLCD t = LocalEV3.get().getTextLCD();
		Odometer odometer = new Odometer(leftMotor, rightMotor);
		Navigation na = new Navigation(odometer,leftMotor, rightMotor, ziplineMotor);

		do {
			// clear the display
			t.clear();
			
			// ask the user whether the motors should drive in the rising edge mode or falling edge mode
			t.drawString("< Please select  >", 0, 0);
			t.drawString("corner number in  ", 0, 1);
			t.drawString("the range [0,3]:  ", 0, 2);
			t.drawString(cornerCounter + "                 ", 0, 3);
			
			buttonChoice = Button.waitForAnyPress();
			if(buttonChoice == Button.ID_LEFT){
				cornerCounter--;
				if (cornerCounter < 0) cornerCounter = 3;
			}
			if(buttonChoice == Button.ID_RIGHT){
				cornerCounter++;
				if (cornerCounter > 3) cornerCounter = 0;
			}
		} while (buttonChoice != Button.ID_ENTER);
		
		do {
			// clear the display
			t.clear();
			
			// ask the user whether the motors should drive in the rising edge mode or falling edge mode
			t.drawString("< Please select  >", 0, 0);
			t.drawString("Xo and Yo in the  ", 0, 1);
			t.drawString("range [0,8]:      ", 0, 2);
			t.drawString(" (" + xoCounter + " " + yoCounter + ") ", 0, 3);
			
			buttonChoice = Button.waitForAnyPress();
			if(buttonChoice == Button.ID_LEFT){
				xoCounter++;
				if (xoCounter > 8) xoCounter = 0;
			}
			if(buttonChoice == Button.ID_RIGHT){
				yoCounter++;
				if (yoCounter > 8) yoCounter = 0;
			}
		} while (buttonChoice != Button.ID_ENTER);
		
		double minDis = Double.MAX_VALUE;
		bestCorner = 4;
		for (int i = 0; i < CORNERS.length; i++){
			double dis = Math.pow(CORNERS[i][0] - xoCounter, 2) + Math.pow(CORNERS[i][1] - yoCounter, 2);
			if (dis < minDis) {
				minDis = dis;
				bestCorner = i;
			}
		}
		do {
			// clear the display
			t.clear();
			
			// ask the user whether the motors should drive in the rising edge mode or falling edge mode
			t.drawString("< Please select  >", 0, 0);
			t.drawString("Xc and Yc in the  ", 0, 1);
			t.drawString("range [0,8]:      ", 0, 2);
			t.drawString(" (" + xcCounter + " " + ycCounter + ") ", 0, 3);
			
			buttonChoice = Button.waitForAnyPress();
			if(buttonChoice == Button.ID_LEFT){
				xcCounter++;
				if (xcCounter > 8) xcCounter = 0;
			}
			if(buttonChoice == Button.ID_RIGHT){
				ycCounter++;
				if (ycCounter > 8) ycCounter = 0;
			}
		} while (buttonChoice != Button.ID_ENTER);
		
		ultraLoc = new UltrasonicLocalizer(0, na, odometer);
		
		odometer.start();
		OdometryDisplay od = new OdometryDisplay(odometer, t,ultraLoc);
		od.start();
		usPoller = new UltrasonicPoller(usDistance, usData, ultraLoc);

		usPoller.start();
		ultraLoc.start();
		
		try {
			ultraLoc.join();
		} catch (InterruptedException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
//		Button.waitForAnyPress();

		try { 
			Thread.sleep(500);
		} catch (Exception e) {
		}
		lightLoc = new LightLocalizer(odometer, na);
		double x = ultraLoc.getLocX();
		double y = ultraLoc.getLocY();
		//System.out.println("Fin US localization");
		//System.out.println("X: "+x+" - Y: "+y);

		/**
		 * Set the x and y to be measured location and make the robot travel to the point (0,0)
		 * Now the robot should roughly be at the point (0,0). Then we start the light localization 
		 * which navigates the robot to the exact (0,0) point.
		 * 
		 */
		odometer.setX(x-SQUARE_LENGTH);
		odometer.setY(y-SQUARE_LENGTH);
		//System.out.println("Set new coordinates minus length");
		//System.out.println("X: "+odometer.getX()+" - Y: "+odometer.getY());
		na.travelTo(3, 3);
		

		//Wait until the robot stops moving, start the lightLocalization thread
		while(na.isNavigating()){
		}
		lightLoc.start();
		while(na.isNavigating()){
		}
		//System.out.println("Are coordinates set to 0 ?");
		//System.out.println("X: "+odometer.getX()+" - Y: "+odometer.getY() + " - Theta: " + odometer.getTheta());
		//Button.waitForAnyPress();
	
		try {
			lightLoc.join();
		} catch (InterruptedException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
		//System.out.println("You are at: X: "+ odometer.getX() +" - Y: " +odometer.getY() + " - Theta: " + odometer.getTheta());
		//System.out.println("travelling to:");//I have no idea why but the travelTo method "inverses" x and y
		if(yoCounter == ycCounter && CORNERS[cornerCounter][0] != xoCounter){
			//na.travelTo(odometer.getY(), xoCounter*SQUARE_LENGTH);
			curX = odometer.getX();
			curY = odometer.getY();
			
			//first light localization
			na.raphTravelTo((xoCounter*SQUARE_LENGTH + curX)/2,curY);
			curTheta = odometer.getTheta();
			while(na.isNavigating()){
				
			}
			odometer.setTheta(90);
			lightLoc = new LightLocalizer(odometer, na);
			lightLoc.doLightLocalization();
			while(na.isNavigating()){
				
			}
			na.makeTurn(90);
			while(na.isNavigating()){
				
			}
			odometer.setX((xoCounter*SQUARE_LENGTH + curX)/2);
			odometer.setY(curY);
			odometer.setTheta(curTheta);
			
			//second light localization
			na.raphTravelTo(xoCounter*SQUARE_LENGTH,curY);
			curX = odometer.getX();
			curY = odometer.getY();
			curTheta = odometer.getTheta();
			odometer.setTheta(90);
			lightLoc = new LightLocalizer(odometer, na);
			lightLoc.doLightLocalization();
			while(na.isNavigating()){
				
			}
			na.makeTurn(90);
			while(na.isNavigating()){
				
			}
			odometer.setX(xoCounter*SQUARE_LENGTH);
			odometer.setY(curY);
			odometer.setTheta(curTheta);

		}else if(xoCounter == xcCounter && CORNERS[cornerCounter][1] != yoCounter){

			curX = odometer.getX();
			curY = odometer.getY();
			curTheta = odometer.getTheta();
			
			//first light localization
			na.raphTravelTo(curX, (yoCounter*SQUARE_LENGTH + curY)/2);
			while(na.isNavigating()){
				
			}
			odometer.setTheta(90);
			lightLoc = new LightLocalizer(odometer, na);
			lightLoc.doLightLocalization();
			while(na.isNavigating()){
				
			}
			na.makeTurn(90);
			while(na.isNavigating()){
				
			}
			odometer.setX(curX);
			odometer.setY((yoCounter*SQUARE_LENGTH + curY)/2);
			odometer.setTheta(curTheta);
			
			//second light localization
			na.raphTravelTo(curX,yoCounter*SQUARE_LENGTH);
			curX = odometer.getX();
			curY = odometer.getY();
			curTheta = odometer.getTheta();
			odometer.setTheta(90);
			lightLoc = new LightLocalizer(odometer, na);
			lightLoc.doLightLocalization();
			while(na.isNavigating()){
				
			}
			na.makeTurn(90);
			while(na.isNavigating()){
				
			}
			odometer.setX(curX);
			odometer.setY(yoCounter*SQUARE_LENGTH);
			odometer.setTheta(curTheta);
		}
		while(na.isNavigating()){
			
		}
		
		//perform correction here
		//System.out.println("finished first travel, the robot is now at: X: "+ odometer.getX() +" - Y: " +odometer.getY());
		//System.out.println("now it is going to: X: "+ xoCounter*SQUARE_LENGTH +" - Y: " + yoCounter*SQUARE_LENGTH);
		
		//The third localization
		if (cornerCounter != bestCorner){
			curX = odometer.getX();
			curY = odometer.getY();
			curTheta = odometer.getTheta();
			na.raphTravelTo((curX + CORNERS[bestCorner][0]*SQUARE_LENGTH)/2, (curY + CORNERS[bestCorner][1]*SQUARE_LENGTH)/2);
			odometer.setTheta(90);
			lightLoc = new LightLocalizer(odometer, na);
			lightLoc.doLightLocalization();
			while(na.isNavigating()){
				
			}
			na.makeTurn(90);
			while(na.isNavigating()){
				
			}
			odometer.setX((curX + CORNERS[bestCorner][0]*SQUARE_LENGTH)/2);
			odometer.setY((curY + CORNERS[bestCorner][1]*SQUARE_LENGTH)/2);
			odometer.setTheta(curTheta);
		}
		//The fourth localization
		na.raphTravelTo(xoCounter*SQUARE_LENGTH,yoCounter*SQUARE_LENGTH);
		while(na.isNavigating()){
			
		}
		curX = odometer.getX();
		curY = odometer.getY();
		curTheta = odometer.getTheta();
		odometer.setTheta(90);
		lightLoc = new LightLocalizer(odometer, na);
		lightLoc.doLightLocalization();
		while(na.isNavigating()){
			
		}
		na.makeTurn(90);
		while(na.isNavigating()){
			
		}
		odometer.setX(curX);
		odometer.setY(curY);
		odometer.setTheta(curTheta);
		
		//hardcode (don't comment)
		if (cornerCounter == 2 || cornerCounter == 3)
			na.makeTurn(-90);
		else 
			na.makeTurn(90);
		//
		
		//should comment
//		if(ycCounter > yoCounter){
//			na.makeTurn(0);
//		}else if(xcCounter > xoCounter){
//			na.makeTurn(90);
//		}else if(ycCounter < yoCounter){
//			na.makeTurn(180);
//		}else if(xcCounter < xoCounter){
//			na.makeTurn(270);
//		}
		while(na.isNavigating()){
			
		}
		
		//hardcode (don't comment)
		na.makeTurn(-7);
		odometer.setTheta(90);
		//
		
		
		while(na.isNavigating()){
			
		}
		Button.waitForAnyPress();
		
		na.raphTravelTo(xcCounter*SQUARE_LENGTH+10,ycCounter*SQUARE_LENGTH);

		//hardcode
	//	odometer.setX(xcCounter*SQUARE_LENGTH+10);
		//odometer.setY(ycCounter*SQUARE_LENGTH);
		na.doZipline(3*ZIPLENGTH);
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}
}
