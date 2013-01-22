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
	
	// range of movement in clinical terms
	private float abductionLShoulder = 0f;
	private float abductionRShoulder = 0f;
	private float adductionLShoulder = 0f;
	private float adductionRShoulder = 0f;
	private float anteversionLShoulder = 0f;
	private float anteversionRShoulder = 0f;
	private float retroversionLShoulder = 0f;
	private float retroversionRShoulder = 0f;
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
		
		abductionLShoulder = _statistics.getAbductionLShoulder();
		abductionRShoulder = _statistics.getAbductionRShoulder();
		adductionLShoulder = _statistics.getAdductionLShoulder();
		adductionRShoulder = _statistics.getAdductionRShoulder();
		anteversionLShoulder = _statistics.getAnteversionLShoulder();
		anteversionRShoulder = _statistics.getAnteversionRShoulder();
		retroversionLShoulder = _statistics.getRetroversionLShoulder();
		retroversionRShoulder = _statistics.getRetroversionRShoulder();

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
			angleLeftLowerArm = skeleton.getAngleLeftLowerArm();
			angleLeftUpperArm = skeleton.getAngleLeftUpperArm();
			angleRightLowerArm = skeleton.getAngleRightLowerArm();
			angleRightUpperArm = skeleton.getAngleRightUpperArm();
			if (angleLeftLowerArm > maxAngleLeftLowerArm) maxAngleLeftLowerArm = angleLeftLowerArm;
			if (PConstants.PI-angleLeftUpperArm > maxAngleLeftUpperArm) maxAngleLeftUpperArm = PConstants.PI-angleLeftUpperArm;
			if (angleRightLowerArm > maxAngleRightLowerArm) maxAngleRightLowerArm = angleRightLowerArm;
			if (PConstants.PI-angleRightUpperArm > maxAngleRightUpperArm) maxAngleRightUpperArm = PConstants.PI-angleRightUpperArm;
			
			// calculation of orthopaedic angles for upper arm (calculation for lower arm is not possible from kinect data)
			abductionLShoulder = skeleton.getAbduction(Skeleton.LEFT_ELBOW,Skeleton.LEFT_SHOULDER);
			abductionRShoulder = skeleton.getAbduction(Skeleton.RIGHT_ELBOW,Skeleton.RIGHT_SHOULDER);
			adductionLShoulder = skeleton.getAdduction(Skeleton.LEFT_ELBOW,Skeleton.LEFT_SHOULDER);
			adductionRShoulder = skeleton.getAdduction(Skeleton.RIGHT_ELBOW,Skeleton.RIGHT_SHOULDER);
			anteversionLShoulder = skeleton.getAnteversion(Skeleton.LEFT_ELBOW,Skeleton.LEFT_SHOULDER);
			anteversionRShoulder = skeleton.getAnteversion(Skeleton.RIGHT_ELBOW,Skeleton.RIGHT_SHOULDER);
			retroversionLShoulder = skeleton.getRetroversion(Skeleton.LEFT_ELBOW,Skeleton.LEFT_SHOULDER);
			retroversionRShoulder = skeleton.getRetroversion(Skeleton.RIGHT_ELBOW,Skeleton.RIGHT_SHOULDER);
			if (abductionLShoulder > maxAbductionLShoulder) maxAbductionLShoulder = abductionLShoulder;
			if (abductionRShoulder > maxAbductionRShoulder) maxAbductionRShoulder = abductionRShoulder;
			if (adductionLShoulder > maxAdductionLShoulder) maxAdductionLShoulder = adductionLShoulder;
			if (adductionRShoulder > maxAdductionRShoulder) maxAdductionRShoulder = adductionRShoulder;
			if (anteversionLShoulder > maxAnteversionLShoulder) maxAnteversionLShoulder = anteversionLShoulder;
			if (anteversionRShoulder > maxAnteversionRShoulder) maxAnteversionRShoulder = anteversionRShoulder;
			if (retroversionLShoulder > maxRetroversionLShoulder) maxRetroversionLShoulder = retroversionLShoulder;
			if (retroversionRShoulder > maxRetroversionRShoulder) maxRetroversionRShoulder = retroversionRShoulder;
			
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

	public float getAbductionLShoulder() {
		return abductionLShoulder;
	}

	public float getAbductionRShoulder() {
		return abductionRShoulder;
	}

	public float getAdductionLShoulder() {
		return adductionLShoulder;
	}

	public float getAdductionRShoulder() {
		return adductionRShoulder;
	}

	public float getAnteversionLShoulder() {
		return anteversionLShoulder;
	}

	public float getAnteversionRShoulder() {
		return anteversionRShoulder;
	}

	public float getRetroversionLShoulder() {
		return retroversionLShoulder;
	}

	public float getRetroversionRShoulder() {
		return retroversionRShoulder;
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
