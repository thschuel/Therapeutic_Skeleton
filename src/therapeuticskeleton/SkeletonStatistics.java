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
	private float maxAngleLeftLowerArm = 0f;
	private float maxAngleLeftUpperArm = 0f;
	private float maxAngleRightLowerArm = 0f;
	private float maxAngleRightUpperArm = 0f;
	
	// range of movement in clinical terms
	private float maxAbductionLShoulder = 0f;
	private float maxAbductionRShoulder = 0f;
	private float maxAdductionLShoulder = 0f;
	private float maxAdductionRShoulder = 0f;
	private float maxAnteversionLShoulder = 0f;
	private float maxAnteversionRShoulder = 0f;
	private float maxRetroversionLShoulder = 0f;
	private float maxRetroversionRShoulder = 0f;
	
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
						"maxAngleRightUpperArm," +
						"maxAbductionLS," +
						"maxAdductionLS," +
						"maxAnteversionLS," +
						"maxRetroversionLS," +
						"maxAbductionRS," +
						"maxAdductionRS," +
						"maxAnteversionRS," +
						"maxRetroversionRS\n");
				buffer.write(""+seconds+","+
						distanceLeftHand+","+
						distanceLeftElbow+","+
						distanceRightHand+","+
						distanceRightElbow+","+
						PApplet.degrees(maxAngleLeftLowerArm)+","+
						PApplet.degrees(maxAngleLeftUpperArm)+","+
						PApplet.degrees(maxAngleRightLowerArm)+","+
						PApplet.degrees(maxAngleRightUpperArm)+","+
						PApplet.degrees(maxAbductionLShoulder)+","+
						PApplet.degrees(maxAdductionLShoulder)+","+
						PApplet.degrees(maxAnteversionLShoulder)+","+
						PApplet.degrees(maxRetroversionLShoulder)+","+
						PApplet.degrees(maxAbductionRShoulder)+","+
						PApplet.degrees(maxAdductionRShoulder)+","+
						PApplet.degrees(maxAnteversionRShoulder)+","+
						PApplet.degrees(maxRetroversionRShoulder));
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
		
		velocityLeftHand = _statistics.getDistancePerSecondLeftHand();
		velocityLeftElbow = _statistics.getDistancePerSecondLeftElbow();
		velocityRightHand = _statistics.getDistancePerSecondRightHand();
		velocityRightElbow = _statistics.getDistancePerSecondRightElbow();
		
		directionOfMovementLeftHand = _statistics.getDirectionOfMovementLeftHand();
		directionOfMovementLeftElbow = _statistics.getDirectionOfMovementLeftElbow();
		directionOfMovementRightHand = _statistics.getDirectionOfMovementRightHand();
		directionOfMovementRightElbow = _statistics.getDirectionOfMovementRightElbow();

		counterConstantMovementLeftHand = _statistics.getCounterConstantMovementLeftHand();
		counterConstantMovementLeftElbow = _statistics.getCounterConstantMovementLeftElbow();
		counterConstantMovementRightHand = _statistics.getCounterConstantMovementRightHand();
		counterConstantMovementRightElbow = _statistics.getCounterConstantMovementRightElbow();
		
		maxAngleLeftLowerArm = _statistics.getMaxAngleLeftLowerArm();
		maxAngleLeftUpperArm = _statistics.getMaxAngleLeftUpperArm();
		maxAngleRightLowerArm = _statistics.getMaxAngleRightLowerArm();
		maxAngleRightUpperArm = _statistics.getMaxAngleRightUpperArm();

		maxAbductionLShoulder = _statistics.getMaxAbductionLShoulder();
		maxAbductionRShoulder = _statistics.getMaxAbductionRShoulder();
		maxAdductionLShoulder = _statistics.getMaxAdductionLShoulder();
		maxAdductionRShoulder = _statistics.getMaxAdductionRShoulder();
		maxAnteversionLShoulder = _statistics.getMaxAnteversionLShoulder();
		maxAnteversionRShoulder = _statistics.getMaxAnteversionRShoulder();
		maxRetroversionLShoulder = _statistics.getMaxRetroversionLShoulder();
		maxRetroversionRShoulder = _statistics.getMaxRetroversionRShoulder();
	}
	
	public void update (int _frameCount, float _frameRate) {
		if (lastFrameCount != -9999) {
			seconds += (_frameCount-lastFrameCount)/_frameRate;
		}
		lastFrameCount = _frameCount;
		if (skeleton != null) {
			// copy joint information to new pvector to store in history
			PVector tempLeftHand = new PVector();
			tempLeftHand.set(skeleton.getJoint(Skeleton.LEFT_HAND));
			PVector tempLeftElbow = new PVector();
			tempLeftElbow.set(skeleton.getJoint(Skeleton.LEFT_ELBOW));
			PVector tempRightHand = new PVector();
			tempRightHand.set(skeleton.getJoint(Skeleton.RIGHT_HAND));
			PVector tempRightElbow = new PVector();
			tempRightElbow.set(skeleton.getJoint(Skeleton.RIGHT_ELBOW));
			
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
			distanceLeftHand += skeleton.getJointDelta(Skeleton.LEFT_HAND);
			distanceLeftElbow += skeleton.getJointDelta(Skeleton.LEFT_ELBOW);
			distanceRightHand += skeleton.getJointDelta(Skeleton.RIGHT_HAND);
			distanceRightElbow += skeleton.getJointDelta(Skeleton.RIGHT_ELBOW);
			
			// calculation of velocity of joints in mm/second
			velocityLeftHand = skeleton.getJointDelta(Skeleton.LEFT_HAND)*_frameRate;
			velocityLeftElbow = skeleton.getJointDelta(Skeleton.LEFT_ELBOW)*_frameRate;
			velocityRightHand = skeleton.getJointDelta(Skeleton.RIGHT_HAND)*_frameRate;
			velocityRightElbow = skeleton.getJointDelta(Skeleton.RIGHT_ELBOW)*_frameRate;
			
			// calculation of max angles
			float tempAngleLeftLowerArm = skeleton.getAngleLeftLowerArm();
			float tempAngleLeftUpperArm = skeleton.getAngleLeftUpperArm();
			float tempAngleRightLowerArm = skeleton.getAngleRightLowerArm();
			float tempAngleRightUpperArm = skeleton.getAngleRightUpperArm();
			if (tempAngleLeftLowerArm > maxAngleLeftLowerArm) maxAngleLeftLowerArm = tempAngleLeftLowerArm;
			if (PConstants.PI-tempAngleLeftUpperArm > maxAngleLeftUpperArm) maxAngleLeftUpperArm = PConstants.PI-tempAngleLeftUpperArm;
			if (tempAngleRightLowerArm > maxAngleRightLowerArm) maxAngleRightLowerArm = tempAngleRightLowerArm;
			if (PConstants.PI-tempAngleRightUpperArm > maxAngleRightUpperArm) maxAngleRightUpperArm = PConstants.PI-tempAngleRightUpperArm;
			
			// calculation of orthopaedic angles for upper arm (calculation for lower arm is not possible from kinect data)
			float tempAbduktionLShoulder = skeleton.getAbduction(Skeleton.LEFT_ELBOW,Skeleton.LEFT_SHOULDER);
			float tempAbduktionRShoulder = skeleton.getAbduction(Skeleton.RIGHT_ELBOW,Skeleton.RIGHT_SHOULDER);
			float tempAdduktionLShoulder = skeleton.getAdduction(Skeleton.LEFT_ELBOW,Skeleton.LEFT_SHOULDER);
			float tempAdduktionRShoulder = skeleton.getAdduction(Skeleton.RIGHT_ELBOW,Skeleton.RIGHT_SHOULDER);
			float tempAnteversionLShoulder = skeleton.getAnteversion(Skeleton.LEFT_ELBOW,Skeleton.LEFT_SHOULDER);
			float tempAnteversionRShoulder = skeleton.getAnteversion(Skeleton.RIGHT_ELBOW,Skeleton.RIGHT_SHOULDER);
			float tempRetroversionLShoulder = skeleton.getRetroversion(Skeleton.LEFT_ELBOW,Skeleton.LEFT_SHOULDER);
			float tempRetroversionRShoulder = skeleton.getRetroversion(Skeleton.RIGHT_ELBOW,Skeleton.RIGHT_SHOULDER);
			if (tempAbduktionLShoulder > maxAbductionLShoulder) maxAbductionLShoulder = tempAbduktionLShoulder;
			if (tempAbduktionRShoulder > maxAbductionRShoulder) maxAbductionRShoulder = tempAbduktionRShoulder;
			if (tempAdduktionLShoulder > maxAdductionLShoulder) maxAdductionLShoulder = tempAdduktionLShoulder;
			if (tempAdduktionRShoulder > maxAdductionRShoulder) maxAdductionRShoulder = tempAdduktionRShoulder;
			if (tempAnteversionLShoulder > maxAnteversionLShoulder) maxAnteversionLShoulder = tempAnteversionLShoulder;
			if (tempAnteversionRShoulder > maxAnteversionRShoulder) maxAnteversionRShoulder = tempAnteversionRShoulder;
			if (tempRetroversionLShoulder > maxRetroversionLShoulder) maxRetroversionLShoulder = tempRetroversionLShoulder;
			if (tempRetroversionRShoulder > maxRetroversionRShoulder) maxRetroversionRShoulder = tempRetroversionRShoulder;
			
			// log information
			if (buffer != null) {
				try {
					buffer.write(""+seconds+","+
								velocityLeftHand+","+
								velocityLeftElbow+","+
								velocityRightHand+","+
								velocityRightElbow+","+
								skeleton.getJointDelta(Skeleton.LEFT_HAND)+","+
								skeleton.getJointDelta(Skeleton.LEFT_ELBOW)+","+
								skeleton.getJointDelta(Skeleton.RIGHT_HAND)+","+
								skeleton.getJointDelta(Skeleton.RIGHT_ELBOW)+","+
								skeleton.getJointLCS(Skeleton.LEFT_HAND).x+","+
								skeleton.getJointLCS(Skeleton.LEFT_HAND).y+","+
								skeleton.getJointLCS(Skeleton.LEFT_HAND).z+","+
								skeleton.getJointLCS(Skeleton.LEFT_ELBOW).x+","+
								skeleton.getJointLCS(Skeleton.LEFT_ELBOW).y+","+
								skeleton.getJointLCS(Skeleton.LEFT_ELBOW).z+","+
								skeleton.getJointLCS(Skeleton.RIGHT_HAND).x+","+
								skeleton.getJointLCS(Skeleton.RIGHT_HAND).y+","+
								skeleton.getJointLCS(Skeleton.RIGHT_HAND).z+","+
								skeleton.getJointLCS(Skeleton.RIGHT_ELBOW).x+","+
								skeleton.getJointLCS(Skeleton.RIGHT_ELBOW).y+","+
								skeleton.getJointLCS(Skeleton.RIGHT_ELBOW).z+"\n");
					
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

	public float getDistancePerSecondLeftHand() {
		return velocityLeftHand;
	}

	public float getDistancePerSecondLeftElbow() {
		return velocityLeftElbow;
	}

	public float getDistancePerSecondRightHand() {
		return velocityRightHand;
	}

	public float getDistancePerSecondRightElbow() {
		return velocityRightElbow;
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

	public float getMaxAbductionLShoulder() {
		return maxAbductionLShoulder;
	}

	public float getMaxAbductionRShoulder() {
		return maxAbductionRShoulder;
	}

	public float getMaxAdductionLShoulder() {
		return maxAdductionLShoulder;
	}

	public float getMaxAdductionRShoulder() {
		return maxAdductionRShoulder;
	}

	public float getMaxAnteversionLShoulder() {
		return maxAnteversionLShoulder;
	}

	public float getMaxAnteversionRShoulder() {
		return maxAnteversionRShoulder;
	}

	public float getMaxRetroversionLShoulder() {
		return maxRetroversionLShoulder;
	}

	public float getMaxRetroversionRShoulder() {
		return maxRetroversionRShoulder;
	}
	
}
