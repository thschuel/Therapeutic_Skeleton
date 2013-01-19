package therapeuticskeleton;

import java.io.BufferedWriter;
import java.util.ArrayList;
import processing.core.*;

public class SkeletonStatistics {
	private ArrayList<PVector> historyLeftHandLCS = new ArrayList<PVector>();
	private ArrayList<PVector> historyLeftElbowLCS = new ArrayList<PVector>();
	private ArrayList<PVector> historyRightHandLCS = new ArrayList<PVector>();
	private ArrayList<PVector> historyRightElbowLCS = new ArrayList<PVector>();
	
	private PVector lastSkeletonPosition = new PVector();
	private PVector lastTorsoLCS = new PVector();
	private PVector lastLeftShoulderLCS = new PVector();
	private PVector lastRightShoulderLCS = new PVector();
	
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
	private PVector lastDirectionOfMovementLeftHand = new PVector();
	private PVector lastDirectionOfMovementLeftElbow = new PVector();
	private PVector lastDirectionOfMovementRightHand = new PVector();
	private PVector lastDirectionOfMovementRightElbow = new PVector();
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
	private float maxAbduktionLShoulder = 0f;
	private float maxAbduktionRShoulder = 0f;
	private float maxAdduktionLShoulder = 0f;
	private float maxAdduktionRShoulder = 0f;
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
				buffer.write("Second,velocityLH,velocityLE,velocityRH,velocityRE,deltaLH,deltaLE,deltaRH,deltaRE\n");				
			} catch (Exception e) {
				PApplet.println("couldn't write to file, buffer exception");
			}
		}
	}
	
	public void stopStatisticsLogging() {
		if (buffer != null) {
			try {
				// write the footer
				buffer.write("\ntime,distanceLH,distanceLE,distanceRH,distanceRE,maxAngleLeftLowerArm,maxAngleLeftUpperArm,maxAngleRightLowerArm,maxAngleRightUpperArm\n");
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
		historyLeftHandLCS.addAll(_statistics.getHistoryLeftHandLCS());
		historyLeftElbowLCS.addAll(_statistics.getHistoryLeftElbowLCS());
		historyRightHandLCS.addAll(_statistics.getHistoryRightHandLCS());
		historyRightElbowLCS.addAll(_statistics.getHistoryRightElbowLCS());
		
		lastSkeletonPosition.set(_statistics.getLastSkeletonPosition());
		lastTorsoLCS.set(_statistics.getLastTorsoLCS());
		lastLeftShoulderLCS.set(_statistics.getLastLeftShoulderLCS());
		lastRightShoulderLCS.set(_statistics.getLastRightShoulderLCS());
		
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

		maxAbduktionLShoulder = _statistics.getMaxAbduktionLShoulder();
		maxAbduktionRShoulder = _statistics.getMaxAbduktionRShoulder();
		maxAdduktionLShoulder = _statistics.getMaxAdduktionLShoulder();
		maxAdduktionRShoulder = _statistics.getMaxAdduktionRShoulder();
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
			PVector tempLeftHandLCS = new PVector();
			tempLeftHandLCS.set(skeleton.getJointLCS(Skeleton.LEFT_HAND));
			PVector tempLeftElbowLCS = new PVector();
			tempLeftElbowLCS.set(skeleton.getJointLCS(Skeleton.LEFT_ELBOW));
			PVector tempRightHandLCS = new PVector();
			tempRightHandLCS.set(skeleton.getJointLCS(Skeleton.RIGHT_HAND));
			PVector tempRightElbowLCS = new PVector();
			tempRightElbowLCS.set(skeleton.getJointLCS(Skeleton.RIGHT_ELBOW));
			
			distanceLeftHand += skeleton.getJointDelta(Skeleton.LEFT_HAND);
			distanceLeftElbow += skeleton.getJointDelta(Skeleton.LEFT_ELBOW);
			distanceRightHand += skeleton.getJointDelta(Skeleton.RIGHT_HAND);
			distanceRightElbow += skeleton.getJointDelta(Skeleton.RIGHT_ELBOW);
			velocityLeftHand = skeleton.getJointDelta(Skeleton.LEFT_HAND)*_frameRate;
			velocityLeftElbow = skeleton.getJointDelta(Skeleton.LEFT_ELBOW)*_frameRate;
			velocityRightHand = skeleton.getJointDelta(Skeleton.RIGHT_HAND)*_frameRate;
			velocityRightElbow = skeleton.getJointDelta(Skeleton.RIGHT_ELBOW)*_frameRate;
			
			lastDirectionOfMovementLeftHand = directionOfMovementLeftHand;
			lastDirectionOfMovementLeftElbow = directionOfMovementLeftElbow;
			lastDirectionOfMovementRightHand = directionOfMovementRightHand;
			lastDirectionOfMovementRightElbow = directionOfMovementRightElbow;

			if (historyLeftHandLCS.size() > 0) {
				directionOfMovementLeftHand = PVector.sub(tempLeftHandLCS,historyLeftHandLCS.get(historyLeftHandLCS.size()-1));
				directionOfMovementLeftElbow = PVector.sub(tempLeftElbowLCS,historyLeftElbowLCS.get(historyLeftElbowLCS.size()-1));
				directionOfMovementRightHand = PVector.sub(tempRightHandLCS,historyRightHandLCS.get(historyRightHandLCS.size()-1));
				directionOfMovementRightElbow = PVector.sub(tempRightElbowLCS,historyRightElbowLCS.get(historyRightElbowLCS.size()-1));
			}
	
			if (PVector.angleBetween(directionOfMovementLeftHand,lastDirectionOfMovementLeftHand) < PConstants.HALF_PI) {
				counterConstantMovementLeftHand++;
			} else {
				counterConstantMovementLeftHand = 0;
			}
			if (PVector.angleBetween(directionOfMovementLeftElbow,lastDirectionOfMovementLeftElbow) < PConstants.HALF_PI) {
				counterConstantMovementLeftElbow++;
			} else {
				counterConstantMovementLeftElbow = 0;
			}
			if (PVector.angleBetween(directionOfMovementRightHand,lastDirectionOfMovementRightHand) < PConstants.HALF_PI) {
				counterConstantMovementRightHand++;
			} else {
				counterConstantMovementRightHand = 0;
			}
			if (PVector.angleBetween(directionOfMovementRightElbow,lastDirectionOfMovementRightElbow) < PConstants.HALF_PI) {
				counterConstantMovementRightElbow++;
			} else {
				counterConstantMovementRightElbow = 0;
			}
			
			float tempAngleLeftLowerArm = skeleton.getAngleLeftLowerArm();
			float tempAngleLeftUpperArm = skeleton.getAngleLeftUpperArm();
			float tempAngleRightLowerArm = skeleton.getAngleRightLowerArm();
			float tempAngleRightUpperArm = skeleton.getAngleRightUpperArm();
			if (tempAngleLeftLowerArm > maxAngleLeftLowerArm) {
				maxAngleLeftLowerArm = tempAngleLeftLowerArm;
			}
			if (PConstants.PI-tempAngleLeftUpperArm > maxAngleLeftUpperArm) {
				maxAngleLeftUpperArm = PConstants.PI-tempAngleLeftUpperArm;
			}
			if (tempAngleRightLowerArm > maxAngleRightLowerArm) {
				maxAngleRightLowerArm = tempAngleRightLowerArm;
			}
			if (PConstants.PI-tempAngleRightUpperArm > maxAngleRightUpperArm) {
				maxAngleRightUpperArm = PConstants.PI-tempAngleRightUpperArm;
			}
			
			// calculation of orthopaedic angles for upper arm (calculation for lower arm is not possible from kinect data)
			float tempAbduktionLShoulder = 0f;
			float tempAbduktionRShoulder = 0f;
			float tempAdduktionLShoulder = 0f;
			float tempAdduktionRShoulder = 0f;
			float tempAnteversionLShoulder = 0f;
			float tempAnteversionRShoulder = 0f;
			float tempRetroversionLShoulder = 0f;
			float tempRetroversionRShoulder = 0f;

			// negative y axis is needed for neutral-zero-method
			PVector negY = new PVector();
			negY.set(skeleton.getOrientationY());
			negY.mult(-1f);
			
			if (skeleton.getOrientationInFrontalPlane(Skeleton.LEFT_ELBOW,Skeleton.LEFT_SHOULDER) == Skeleton.LEFT_LATERAL) {
				tempAbduktionLShoulder = PVector.angleBetween(skeleton.projectionOnFrontalPlane(Skeleton.LEFT_ELBOW,Skeleton.LEFT_SHOULDER),negY);
			} else {
				tempAdduktionLShoulder = PVector.angleBetween(skeleton.projectionOnFrontalPlane(Skeleton.LEFT_ELBOW,Skeleton.LEFT_SHOULDER),negY);
			}
			if (skeleton.getOrientationInFrontalPlane(Skeleton.RIGHT_ELBOW,Skeleton.RIGHT_SHOULDER) == Skeleton.RIGHT_LATERAL) {
				tempAbduktionRShoulder = PVector.angleBetween(skeleton.projectionOnFrontalPlane(Skeleton.RIGHT_ELBOW,Skeleton.RIGHT_SHOULDER),negY);
			} else {
				tempAdduktionRShoulder = PVector.angleBetween(skeleton.projectionOnFrontalPlane(Skeleton.RIGHT_ELBOW,Skeleton.RIGHT_SHOULDER),negY);
			}
			if (skeleton.getOrientationInSagittalPlane(Skeleton.LEFT_ELBOW,Skeleton.LEFT_SHOULDER) == Skeleton.ANTERIOR) {
				tempAnteversionLShoulder = PVector.angleBetween(skeleton.projectionOnSagittalPlane(Skeleton.LEFT_ELBOW,Skeleton.LEFT_SHOULDER),negY);
			} else {
				tempRetroversionLShoulder = PVector.angleBetween(skeleton.projectionOnSagittalPlane(Skeleton.LEFT_ELBOW,Skeleton.LEFT_SHOULDER),negY);
			}
			if (skeleton.getOrientationInSagittalPlane(Skeleton.RIGHT_ELBOW,Skeleton.RIGHT_SHOULDER) == Skeleton.ANTERIOR) {
				tempAnteversionRShoulder = PVector.angleBetween(skeleton.projectionOnSagittalPlane(Skeleton.RIGHT_ELBOW,Skeleton.RIGHT_SHOULDER),negY);
			} else {
				tempRetroversionRShoulder = PVector.angleBetween(skeleton.projectionOnSagittalPlane(Skeleton.RIGHT_ELBOW,Skeleton.RIGHT_SHOULDER),negY);
			}
			
			if (tempAbduktionLShoulder > maxAbduktionLShoulder) {
				maxAbduktionLShoulder = tempAbduktionLShoulder;
			}
			if (tempAbduktionRShoulder > maxAbduktionRShoulder) {
				maxAbduktionRShoulder = tempAbduktionRShoulder;
			}
			if (tempAdduktionLShoulder > maxAdduktionLShoulder) {
				maxAdduktionLShoulder = tempAdduktionLShoulder;
			}
			if (tempAdduktionRShoulder > maxAdduktionRShoulder) {
				maxAdduktionRShoulder = tempAdduktionRShoulder;
			}
			if (tempAnteversionLShoulder > maxAnteversionLShoulder) {
				maxAnteversionLShoulder = tempAnteversionLShoulder;
			}
			if (tempAnteversionRShoulder > maxAnteversionRShoulder) {
				maxAnteversionRShoulder = tempAnteversionRShoulder;
			}
			if (tempRetroversionLShoulder > maxRetroversionLShoulder) {
				maxRetroversionLShoulder = tempRetroversionLShoulder;
			}
			if (tempRetroversionRShoulder > maxRetroversionRShoulder) {
				maxRetroversionRShoulder = tempRetroversionRShoulder;
			}
			
			historyLeftHandLCS.add(tempLeftHandLCS);
			historyLeftElbowLCS.add(tempLeftElbowLCS);
			historyRightHandLCS.add(tempRightHandLCS);
			historyRightElbowLCS.add(tempRightElbowLCS);
			
			lastSkeletonPosition.set(skeleton.getOrigin());
			lastTorsoLCS.set(skeleton.getJointLCS(Skeleton.TORSO));
			lastLeftShoulderLCS.set(skeleton.getJointLCS(Skeleton.LEFT_SHOULDER));
			lastRightShoulderLCS.set(skeleton.getJointLCS(Skeleton.RIGHT_SHOULDER));
			
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
								skeleton.getJointDelta(Skeleton.RIGHT_ELBOW)+"\n");
					
				} catch (Exception e) {
					PApplet.println("couldn't write to file, buffer exception");
				}
			}
			
		}
	}

	public ArrayList<PVector> getHistoryLeftHandLCS() {
		return historyLeftHandLCS;
	}

	public ArrayList<PVector> getHistoryLeftElbowLCS() {
		return historyLeftElbowLCS;
	}

	public ArrayList<PVector> getHistoryRightHandLCS() {
		return historyRightHandLCS;
	}

	public ArrayList<PVector> getHistoryRightElbowLCS() {
		return historyRightElbowLCS;
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

	public PVector getLastSkeletonPosition() {
		return lastSkeletonPosition;
	}

	public PVector getLastTorsoLCS() {
		return lastTorsoLCS;
	}

	public PVector getLastLeftShoulderLCS() {
		return lastLeftShoulderLCS;
	}

	public PVector getLastRightShoulderLCS() {
		return lastRightShoulderLCS;
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

	public PVector getLastDirectionOfMovementLeftHand() {
		return lastDirectionOfMovementLeftHand;
	}

	public PVector getLastDirectionOfMovementLeftElbow() {
		return lastDirectionOfMovementLeftElbow;
	}

	public PVector getLastDirectionOfMovementRightHand() {
		return lastDirectionOfMovementRightHand;
	}

	public PVector getLastDirectionOfMovementRightElbow() {
		return lastDirectionOfMovementRightElbow;
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

	public float getMaxAbduktionLShoulder() {
		return maxAbduktionLShoulder;
	}

	public float getMaxAbduktionRShoulder() {
		return maxAbduktionRShoulder;
	}

	public float getMaxAdduktionLShoulder() {
		return maxAdduktionLShoulder;
	}

	public float getMaxAdduktionRShoulder() {
		return maxAdduktionRShoulder;
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
