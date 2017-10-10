package ca.mcgill.ecse211.lab4;

import lejos.hardware.Sound;

public class UltrasonicLocalizer extends Thread{
	private int choice;
	private Navigation na;
	private Odometer odometer;

	private int distance;

	private double firstAngle;
	private double secondAngle;
	private double correctedTheta;

	private boolean firstPointDetected;
	private boolean secondPointDetected;

	private double locationX;
	private double locationY;
	private double locationTheta;

	public UltrasonicLocalizer(int choice, Navigation navigation, Odometer odometer) {
		this.choice = choice;
		this.na = navigation;
		this.odometer = odometer;

		this.distance = 0;
		this.correctedTheta = 0;
		this.firstPointDetected = false;
		this.secondPointDetected = false;
		
		this.locationX = 0;
		this.locationY = 0;
	}
	public void processUSData(int distance) {
		this.distance = distance;
	}

	public void run(){
		na.makeTurn(360);

		while (distance == 1){

		}
		if (choice == 0) risingEdge();
		else fallingEdge();
	}

	private void fallingEdge(){
		double theta1 = 0;
		double theta2 = 0;
		while(distance > (LocalizationLab.DISTANCE_THRESHOLD + LocalizationLab.DISTANCE_MARGIN+1)){

		}
		System.out.println("   " + odometer.getTheta());
		if(!firstPointDetected){
			theta1 = odometer.getTheta();
		
			if (distance <= (LocalizationLab.DISTANCE_THRESHOLD - LocalizationLab.DISTANCE_MARGIN - 1)){
				theta2 = odometer.getTheta();
				Sound.playNote(Sound.FLUTE, 440, 250);
			}
			firstAngle = (theta1 + theta2) / 2;
			firstPointDetected = true;
			System.out.println("the firstPoint is" + firstAngle);
		}
		na.motorStop();
		na.makeTurn(-360);

		try {
			Thread.sleep(2000);
		} catch (Exception e) {
		}

		while(distance > (LocalizationLab.DISTANCE_THRESHOLD + LocalizationLab.DISTANCE_MARGIN+1)){

		}
		if (!secondPointDetected){
			theta1 = odometer.getTheta();

			if (distance <= (LocalizationLab.DISTANCE_THRESHOLD - LocalizationLab.DISTANCE_MARGIN - 1)){
				theta2 = odometer.getTheta();
				Sound.playNote(Sound.FLUTE, 440, 250);
			}
			secondPointDetected = true;
			secondAngle = (theta1 + theta2) / 2;
			System.out.println("the secondPoint is" + secondAngle);
		}
		na.motorStop();
		correctAngleAndLocation();
	}

	private void risingEdge(){
		double theta1 = 0;
		double theta2 = 0;

		while(distance < (LocalizationLab.DISTANCE_THRESHOLD - LocalizationLab.DISTANCE_MARGIN-1)){
		}
		if (!firstPointDetected){
			theta1 = odometer.getTheta();
			if (distance >= (LocalizationLab.DISTANCE_THRESHOLD + LocalizationLab.DISTANCE_MARGIN + 1)){
				theta2 = odometer.getTheta();
				Sound.playNote(Sound.FLUTE, 440, 250);
			}

			firstAngle = (theta1 + theta2) / 2;
			firstPointDetected = true;
			System.out.println("the secondPoint is" + firstAngle);

		}
		na.motorStop();
		na.makeTurn(-360);

		try {
			Thread.sleep(2000);
		} catch (Exception e) {
		}

		while(distance < (LocalizationLab.DISTANCE_THRESHOLD - LocalizationLab.DISTANCE_MARGIN-1)){
		}
		if(!secondPointDetected){
			theta1 = odometer.getTheta();

			if (distance >= (LocalizationLab.DISTANCE_THRESHOLD + LocalizationLab.DISTANCE_MARGIN + 1)){
				theta2 = odometer.getTheta();
				Sound.playNote(Sound.FLUTE, 440, 250);
			}
			secondAngle = (theta1 + theta2) / 2;
			secondPointDetected = true;
			System.out.println("the secondPoint is" + secondAngle);
		}
		na.motorStop();
		correctAngleAndLocation();
	}

	public void correctAngleAndLocation(){		  
		locationTheta = (firstAngle + (360 - secondAngle) - 90) / 2 ;
		System.out.println("We are in correction. the locaitonTheta is" + locationTheta);
		if (choice == 0){
			locationX = LocalizationLab.DISTANCE_THRESHOLD * Math.cos(Math.toRadians(locationTheta));
			locationY = LocalizationLab.DISTANCE_THRESHOLD * Math.cos(Math.toRadians(locationTheta));
		}else if (choice == 1){
			locationX = LocalizationLab.DISTANCE_THRESHOLD * Math.sin(Math.toRadians(locationTheta));
			locationY = LocalizationLab.DISTANCE_THRESHOLD * Math.sin(Math.toRadians(locationTheta));
		}

		firstAngle = ((firstAngle % 360) + 360) % 360;
		secondAngle = ((secondAngle % 360) + 360) % 360;

		if (choice == 0) {
			correctedTheta = 45 - ((firstAngle + secondAngle) / 2);
		} else {
			correctedTheta = 225 - ((firstAngle + secondAngle) / 2);
		}

		Double t = odometer.getTheta();
		correctedTheta += t;
		correctedTheta = ((correctedTheta % 360) + 360) % 360;

		
		odometer.setTheta(correctedTheta);	
		
		na.makeCorrectedTurn(-correctedTheta);
		System.out.println("THIS IS IMPORTANT" + correctedTheta + "   " + odometer.getTheta());
	}

	public int readUSDistance() {
		if (this.distance > 300)
			this.distance = 300;
		return this.distance;
	}
	
	public double getLocX(){
		return this.locationX;
	}
	public double getLocY(){
		return this.locationY;
	}

}

