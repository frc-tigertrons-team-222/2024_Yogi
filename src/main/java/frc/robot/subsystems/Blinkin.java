package frc.robot.subsystems;
    

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class Blinkin {
	
	Spark blinkin;

	public Blinkin() {
		blinkin = new Spark(0);
	}
		
	/**
	 * if the robot is not in hatMode and in normal drive, the LED turns solid white (0.93)
	 */
	public void limelightSighted() {
		blinkin.set(-0.37);
	}
    public void noNoteLoaded() {
		blinkin.set(0.99);
	}
    public void noteLoaded() {
		blinkin.set(0.65);
	}
}
