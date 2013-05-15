package therapeuticskeleton;

import java.io.BufferedWriter;
import java.util.ArrayList;
import processing.core.*;

public class SkeletonStatistics {
	private ArrayList<PVector> historyLeftHand = new ArrayList<PVector>();
	private ArrayList<PVector> historyLeftElbow = new ArrayList<PVector>();
	private ArrayList<PVector> historyRightHand = new ArrayList<PVector>();
	private ArrayList<PVector> historyRightElbow = new ArrayList<PVector>();
	
	// overall distance of joints
	private float distanceLeftHand = 0f;
	private float distanceLeftElbow = 0f;
	private float distanceRightHand = 0f;
	private float distanceRightElbow = 0f;
	
	// velocity in distance per second
	private float velocityLeftHand = 0f;
	private float velocityLeftElbow = 0f;
	private float velocityRightHand = 0f;
	private float velocityRightElbow = 0f;

	// to calculate smoothness of movement
	private PVector directionOfMovementLeftHand = new PVector();
	private PVector directionOfMovementLeftElbow = new PVector();
	private PVector directionOfMovementRightHand = new PVector();
	private PVector directionOfMovementRightElbow = new PVector();
	private int counterConstantMovementLeftHand = 0;
	private int counterConstantMovementLeftElbow = 0;
	private int counterConstantMovementRightHand = 0;
	private int counterConstantMovementRightElbow = 0;
	
	// range of movement
	private float angleLeftLowerArm = 0f;
	private float angleLeftUpperArm = 0f;
	private float angleRightLowerArm = 0f;
	private float angleRightUpperArm = 0f;
	private float maxAngleLeftLowerArm = 0f;
	private float maxAngleLeftUpperArm = 0f;
	private float maxAngleRightLowerArm = 0f;
	private float maxAngleRightUpperArm = 0f;
	
	// internal variables
	private Skeleton skeleton = null;
	private BufferedWriter buffer = null;
	private int lastFrameCount = -9999;
	private float seconds = 0.0f; // for logfile
	
	public SkeletonStatistics (Skeleton _skeleton) {
		skeleton = _skeleton;
	}
	
	public void startStatisticsLogging(BufferedWriter _buffer) {
		buffer = _buffer;
		if (buffer != null) {
			try {
				// write the header
				buffer.write("Second," +
						"velocityLH," +
						"velocityLE," +
						"velocityRH," +
						"velocityRE," +
						"deltaLH," +
						"deltaLE," +
						"deltaRH," +
						"deltaRE," +
						"xLH,yLH,zLH," +
						"xLE,yLE,zLE," +
						"xRH,yRH,zRH," +
						"xRE,yRE,zRE\n");				
			} catch (Exception e) {
				PApplet.println("couldn't write to file, buffer exception");
			}
		}
	}
	
	public void stopStatisticsLogging() {
		if (buffer != null) {
			try {
				// write the footer
				buffer.write("\ntime," +
						"distanceLH," +
						"distanceLE," +
						"distanceRH," +
						"distanceRE," +
						"maxAngleLeftLowerArm," +
						"maxAngleLeftUpperArm," +
						"maxAngleRightLowerArm," +
						"maxAngleRightUpperArm\n");
				buffer.write(""+seconds+","+
						distanceLeftHand+","+
						distanceLeftElbow+","+
						distanceRightHand+","+
						distanceRightElbow+","+
						PApplet.degrees(maxAngleLeftLowerArm)+","+
						PApplet.degrees(maxAngleLeftUpperArm)+","+
						PApplet.degrees(maxAngleRightLowerArm)+","+
						PApplet.degrees(maxAngleRightUpperArm));
			} catch (Exception e) {
				PApplet.println("couldn't write to file, buffer exception");
			}
		}
	}
	
	/** Constructor that provides a deep copy of SkeletonStatistics. Only the linked Skeleton is not copied. 
	 *  Can be used to get access to a not anymore updated copy at a given time of statistics. */
	public SkeletonStatistics (SkeletonStatistics _statistics) {
		historyLeftHand.addAll(_statistics.getHistoryLeftHand());
		historyLeftElbow.addAll(_statistics.getHistoryLeftElbow());
		historyRightHand.addAll(_statistics.getHistoryRightHand());
		historyRightElbow.addAll(_statistics.getHistoryRightElbow());
		
		distanceLeftHand = _statistics.getDistanceLeftHand();
		distanceLeftElbow = _statistics.getDistanceLeftElbow();
		distanceRightHand = _statistics.getDistanceRightHand();
		distanceRightElbow = _statistics.getDistanceRightElbow();
		
		velocityLeftHand = _statistics.getVelocityLeftHand();
		velocityLeftElbow = _statistics.getVelocityLeftElbow();
		velocityRightHand = _statistics.getVelocityRightHand();
		velocityRightElbow = _statistics.getVelocityRightElbow();
		
		directionOfMovementLeftHand = _statistics.getDirectionOfMovementLeftHand();
		directionOfMovementLeftElbow = _statistics.getDirectionOfMovementLeftElbow();
		directionOfMovementRightHand = _statistics.getDirectionOfMovementRightHand();
		directionOfMovementRightElbow = _statistics.getDirectionOfMovementRightElbow();

		counterConstantMovementLeftHand = _statistics.getCounterConstantMovementLeftHand();
		counterConstantMovementLeftElbow = _statistics.getCounterConstantMovementLeftElbow();
		counterConstantMovementRightHand = _statistics.getCounterConstantMovementRightHand();
		counterConstantMovementRightElbow = _statistics.getCounterConstantMovementRightElbow();

		angleLeftLowerArm = _statistics.getAngleLeftLowerArm();
		angleLeftUpperArm = _statistics.getAngleLeftUpperArm();
		angleRightLowerArm = _statistics.getAngleRightLowerArm();
		angleRightUpperArm = _statistics.getAngleRightUpperArm();
		
		maxAngleLeftLowerArm = _statistics.getMaxAngleLeftLowerArm();
		maxAngleLeftUpperArm = _statistics.getMaxAngleLeftUpperArm();
		maxAngleRightLowerArm = _statistics.getMaxAngleRightLowerArm();
		maxAngleRightUpperArm = _statistics.getMaxAngleRightUpperArm();
	}
	
	public void update (int _frameCount, float _frameRate) {
		if (lastFrameCount != -9999) {
			seconds += (_frameCount-lastFrameCount)/_frameRate;
		}
		lastFrameCount = _frameCount;
		if (skeleton != null) {
			// copy joint information to new pvector to store in history
			PVector tempLeftHand = new PVector();
			tempLeftHand.set(skeleton.getJointUnmirrored(Skeleton.LEFT_HAND));
			PVector tempLeftElbow = new PVector();
			tempLeftElbow.set(skeleton.getJointUnmirrored(Skeleton.LEFT_ELBOW));
			PVector tempRightHand = new PVector();
			tempRightHand.set(skeleton.getJointUnmirrored(Skeleton.RIGHT_HAND));
			PVector tempRightElbow = new PVector();
			tempRightElbow.set(skeleton.getJointUnmirrored(Skeleton.RIGHT_ELBOW));
			
			// calculation of constant movement indicators
			if (historyLeftHand.size() > 0) {
				PVector lastDirectionOfMovementLeftHand = directionOfMovementLeftHand;
				PVector lastDirectionOfMovementLeftElbow = directionOfMovementLeftElbow;
				PVector lastDirectionOfMovementRightHand = directionOfMovementRightHand;
				PVector lastDirectionOfMovementRightElbow = directionOfMovementRightElbow;
				directionOfMovementLeftHand = PVector.sub(tempLeftHand,historyLeftHand.get(historyLeftHand.size()-1));
				directionOfMovementLeftElbow = PVector.sub(tempLeftElbow,historyLeftElbow.get(historyLeftElbow.size()-1));
				directionOfMovementRightHand = PVector.sub(tempRightHand,historyRightHand.get(historyRightHand.size()-1));
				directionOfMovementRightElbow = PVector.sub(tempRightElbow,historyRightElbow.get(historyRightElbow.size()-1));
				if (PVector.angleBetween(directionOfMovementLeftHand,lastDirectionOfMovementLeftHand) < PConstants.HALF_PI) counterConstantMovementLeftHand++;
				else counterConstantMovementLeftHand = 0;
				if (PVector.angleBetween(directionOfMovementLeftElbow,lastDirectionOfMovementLeftElbow) < PConstants.HALF_PI) counterConstantMovementLeftElbow++;
				else counterConstantMovementLeftElbow = 0;
				if (PVector.angleBetween(directionOfMovementRightHand,lastDirectionOfMovementRightHand) < PConstants.HALF_PI) counterConstantMovementRightHand++;
				else counterConstantMovementRightHand = 0;
				if (PVector.angleBetween(directionOfMovementRightElbow,lastDirectionOfMovementRightElbow) < PConstants.HALF_PI) counterConstantMovementRightElbow++;
				else counterConstantMovementRightElbow = 0;
			}

			// store hand and elbow points to draw history
			historyLeftHand.add(tempLeftHand);
			historyLeftElbow.add(tempLeftElbow);
			historyRightHand.add(tempRightHand);
			historyRightElbow.add(tempRightElbow);

			// accumulation of distance of joints 
			distanceLeftHand += skeleton.getJointDeltaUnmirrored(Skeleton.LEFT_HAND);
			distanceLeftElbow += skeleton.getJointDeltaUnmirrored(Skeleton.LEFT_ELBOW);
			distanceRightHand += skeleton.getJointDeltaUnmirrored(Skeleton.RIGHT_HAND);
			distanceRightElbow += skeleton.getJointDeltaUnmirrored(Skeleton.RIGHT_ELBOW);
			
			// calculation of velocity of joints in mm/second
			velocityLeftHand = skeleton.getJointDeltaUnmirrored(Skeleton.LEFT_HAND)*_frameRate;
			velocityLeftElbow = skeleton.getJointDeltaUnmirrored(Skeleton.LEFT_ELBOW)*_frameRate;
			velocityRightHand = skeleton.getJointDeltaUnmirrored(Skeleton.RIGHT_HAND)*_frameRate;
			velocityRightElbow = skeleton.getJointDeltaUnmirrored(Skeleton.RIGHT_ELBOW)*_frameRate;
			
			// calculation of max angles
			angleLeftLowerArm = skeleton.getAngleLeftLowerArmUnmirrored();
			angleLeftUpperArm = skeleton.getAngleLeftUpperArmUnmirrored();
			angleRightLowerArm = skeleton.getAngleRightLowerArmUnmirrored();
			angleRightUpperArm = skeleton.getAngleRightUpperArmUnmirrored();
			if (angleLeftLowerArm > maxAngleLeftLowerArm) maxAngleLeftLowerArm = angleLeftLowerArm;
			if (PConstants.PI-angleLeftUpperArm > maxAngleLeftUpperArm) maxAngleLeftUpperArm = PConstants.PI-angleLeftUpperArm;
			if (angleRightLowerArm > maxAngleRightLowerArm) maxAngleRightLowerArm = angleRightLowerArm;
			if (PConstants.PI-angleRightUpperArm > maxAngleRightUpperArm) maxAngleRightUpperArm = PConstants.PI-angleRightUpperArm;
			
			
			// log information
			if (buffer != null) {
				try {
					buffer.write(""+seconds+","+
								velocityLeftHand+","+
								velocityLeftElbow+","+
								velocityRightHand+","+
								velocityRightElbow+","+
								skeleton.getJointDeltaUnmirrored(Skeleton.LEFT_HAND)+","+
								skeleton.getJointDeltaUnmirrored(Skeleton.LEFT_ELBOW)+","+
								skeleton.getJointDeltaUnmirrored(Skeleton.RIGHT_HAND)+","+
								skeleton.getJointDeltaUnmirrored(Skeleton.RIGHT_ELBOW)+","+
								skeleton.getJointLCSUnmirrored(Skeleton.LEFT_HAND).x+","+
								skeleton.getJointLCSUnmirrored(Skeleton.LEFT_HAND).y+","+
								skeleton.getJointLCSUnmirrored(Skeleton.LEFT_HAND).z+","+
								skeleton.getJointLCSUnmirrored(Skeleton.LEFT_ELBOW).x+","+
								skeleton.getJointLCSUnmirrored(Skeleton.LEFT_ELBOW).y+","+
								skeleton.getJointLCSUnmirrored(Skeleton.LEFT_ELBOW).z+","+
								skeleton.getJointLCSUnmirrored(Skeleton.RIGHT_HAND).x+","+
								skeleton.getJointLCSUnmirrored(Skeleton.RIGHT_HAND).y+","+
								skeleton.getJointLCSUnmirrored(Skeleton.RIGHT_HAND).z+","+
								skeleton.getJointLCSUnmirrored(Skeleton.RIGHT_ELBOW).x+","+
								skeleton.getJointLCSUnmirrored(Skeleton.RIGHT_ELBOW).y+","+
								skeleton.getJointLCSUnmirrored(Skeleton.RIGHT_ELBOW).z+"\n");
					
				} catch (Exception e) {
					PApplet.println("couldn't write to file, buffer exception");
				}
			}
			
		}
	}

	public ArrayList<PVector> getHistoryLeftHand() {
		return historyLeftHand;
	}

	public ArrayList<PVector> getHistoryLeftElbow() {
		return historyLeftElbow;
	}

	public ArrayList<PVector> getHistoryRightHand() {
		return historyRightHand;
	}

	public ArrayList<PVector> getHistoryRightElbow() {
		return historyRightElbow;
	}

	public float getDistanceLeftHand() {
		return distanceLeftHand;
	}

	public float getDistanceLeftElbow() {
		return distanceLeftElbow;
	}

	public float getDistanceRightHand() {
		return distanceRightHand;
	}

	public float getDistanceRightElbow() {
		return distanceRightElbow;
	}

	public float getMaxAngleLeftLowerArm() {
		return maxAngleLeftLowerArm;
	}

	public float getMaxAngleLeftUpperArm() {
		return maxAngleLeftUpperArm;
	}

	public float getMaxAngleRightLowerArm() {
		return maxAngleRightLowerArm;
	}

	public float getMaxAngleRightUpperArm() {
		return maxAngleRightUpperArm;
	}

	public Skeleton getSkeleton() {
		return skeleton;
	}

	public PVector getDirectionOfMovementLeftHand() {
		return directionOfMovementLeftHand;
	}

	public PVector getDirectionOfMovementLeftElbow() {
		return directionOfMovementLeftElbow;
	}

	public PVector getDirectionOfMovementRightHand() {
		return directionOfMovementRightHand;
	}

	public PVector getDirectionOfMovementRightElbow() {
		return directionOfMovementRightElbow;
	}

	public int getCounterConstantMovementLeftHand() {
		return counterConstantMovementLeftHand;
	}

	public int getCounterConstantMovementLeftElbow() {
		return counterConstantMovementLeftElbow;
	}

	public int getCounterConstantMovementRightHand() {
		return counterConstantMovementRightHand;
	}

	public int getCounterConstantMovementRightElbow() {
		return counterConstantMovementRightElbow;
	}

	public float getVelocityLeftHand() {
		return velocityLeftHand;
	}

	public float getVelocityLeftElbow() {
		return velocityLeftElbow;
	}

	public float getVelocityRightHand() {
		return velocityRightHand;
	}

	public float getVelocityRightElbow() {
		return velocityRightElbow;
	}

	public float getSeconds() {
		return seconds;
	}

	public float getAngleLeftLowerArm() {
		return angleLeftLowerArm;
	}

	public float getAngleLeftUpperArm() {
		return angleLeftUpperArm;
	}

	public float getAngleRightLowerArm() {
		return angleRightLowerArm;
	}

	public float getAngleRightUpperArm() {
		return angleRightUpperArm;
	}
	
}
