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


	public UltrasonicLocalizer(int choice, Navigation navigation, Odometer odometer) {
		this.choice = choice;
		this.na = navigation;
		this.odometer = odometer;

		this.distance = 0;
		this.correctedTheta = 0;
		this.firstPointDetected = false;
		this.secondPointDetected = false;
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
		if(!firstPointDetected){
			theta1 = odometer.getTheta();

			if (distance <= (LocalizationLab.DISTANCE_THRESHOLD - LocalizationLab.DISTANCE_MARGIN - 1)){
				theta2 = odometer.getTheta();
				Sound.playNote(Sound.FLUTE, 440, 250);
			}
			System.out.println("Theta1 is :" + theta1);
			System.out.println("Theta2 is :" + theta2);

			firstAngle = (theta1 + theta2) / 2;
			firstPointDetected = true;
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
		}
		na.motorStop();
		correctAngle();
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
			}

			firstAngle = (theta1 + theta2) / 2;
			System.out.println("Theta1 is :" + theta1);
			System.out.println("Theta2 is :" + theta2);

			System.out.println("firstAngle is: " + firstAngle);
		}
		Sound.playNote(Sound.FLUTE, 440, 250);
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
				na.motorStop();
			}
			secondAngle = (theta1 + theta2) / 2;
			System.out.println("secondAngle is: " + secondAngle);
		}
		Sound.playNote(Sound.FLUTE, 440, 250);

		correctAngle();
	}

	public void correctAngle(){		  
		firstAngle = ((firstAngle % 360) + 360) % 360;
		secondAngle = ((secondAngle % 360) + 360) % 360;

		if (choice == 0) {
			correctedTheta = 45 - ((firstAngle + secondAngle) / 2);
		} else {
			correctedTheta = 225 - ((firstAngle + secondAngle) / 2);
		}

		Double t = odometer.getTheta();
		System.out.println("The odometer reading is " + t);
		correctedTheta += t;
		correctedTheta = ((correctedTheta % 360) + 360) % 360;

		
		odometer.setTheta(correctedTheta);
		System.out.println("The corrected theta is: "+ correctedTheta);
		
		na.makeCorrectedTurn(-correctedTheta);
	}

	public int readUSDistance() {
		if (this.distance > 300)
			this.distance = 300;
		return this.distance;
	}

}

