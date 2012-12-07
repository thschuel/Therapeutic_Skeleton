package therapeuticskeleton;

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
	
	private float distanceLeftHand = 0f;
	private float distanceLeftElbow = 0f;
	private float distanceRightHand = 0f;
	private float distanceRightElbow = 0f;
	
	private float distancePerSecondLeftHand = 0f;
	private float distancePerSecondLeftElbow = 0f;
	private float distancePerSecondRightHand = 0f;
	private float distancePerSecondRightElbow = 0f;

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
	
	private float maxAngleLeftLowerArm = 0f;
	private float maxAngleLeftUpperArm = 0f;
	private float maxAngleRightLowerArm = 0f;
	private float maxAngleRightUpperArm = 0f;
	private int maxAngleLeftLowerArmIndex = 0;
	private int maxAngleLeftUpperArmIndex = 0;
	private int maxAngleRightLowerArmIndex = 0;
	private int maxAngleRightUpperArmIndex = 0;
	
	private Skeleton skeleton = null;
	
	public SkeletonStatistics (Skeleton _skeleton) {
		skeleton = _skeleton;
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
		
		distancePerSecondLeftHand = _statistics.getDistancePerSecondLeftHand();
		distancePerSecondLeftElbow = _statistics.getDistancePerSecondLeftElbow();
		distancePerSecondRightHand = _statistics.getDistancePerSecondRightHand();
		distancePerSecondRightElbow = _statistics.getDistancePerSecondRightElbow();
		
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
		maxAngleLeftLowerArmIndex = _statistics.getMaxAngleLeftLowerArmIndex();
		maxAngleLeftUpperArmIndex = _statistics.getMaxAngleLeftUpperArmIndex();
		maxAngleRightLowerArmIndex = _statistics.getMaxAngleRightLowerArmIndex();
		maxAngleRightUpperArmIndex = _statistics.getMaxAngleRightUpperArmIndex();
	}
	
	public void update () {
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
			distancePerSecondLeftHand = skeleton.getJointDelta(Skeleton.LEFT_HAND)*skeleton.getFrameRate();
			distancePerSecondLeftElbow = skeleton.getJointDelta(Skeleton.LEFT_ELBOW)*skeleton.getFrameRate();
			distancePerSecondRightHand = skeleton.getJointDelta(Skeleton.RIGHT_HAND)*skeleton.getFrameRate();
			distancePerSecondRightElbow = skeleton.getJointDelta(Skeleton.RIGHT_ELBOW)*skeleton.getFrameRate();
			
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
				maxAngleLeftLowerArmIndex = historyLeftHandLCS.size();
			}
			if (PConstants.PI-tempAngleLeftUpperArm > maxAngleLeftUpperArm) {
				maxAngleLeftUpperArm = PConstants.PI-tempAngleLeftUpperArm;
				maxAngleLeftUpperArmIndex = historyLeftElbowLCS.size();
			}
			if (tempAngleRightLowerArm > maxAngleRightLowerArm) {
				maxAngleRightLowerArm = tempAngleRightLowerArm;
				maxAngleRightLowerArmIndex = historyRightHandLCS.size();
			}
			if (PConstants.PI-tempAngleRightUpperArm > maxAngleRightUpperArm) {
				maxAngleRightUpperArm = PConstants.PI-tempAngleRightUpperArm;
				maxAngleRightUpperArmIndex = historyRightElbowLCS.size();
			}
			
			historyLeftHandLCS.add(tempLeftHandLCS);
			historyLeftElbowLCS.add(tempLeftElbowLCS);
			historyRightHandLCS.add(tempRightHandLCS);
			historyRightElbowLCS.add(tempRightElbowLCS);
			
			lastSkeletonPosition.set(skeleton.getOrigin());
			lastTorsoLCS.set(skeleton.getJointLCS(Skeleton.TORSO));
			lastLeftShoulderLCS.set(skeleton.getJointLCS(Skeleton.LEFT_SHOULDER));
			lastRightShoulderLCS.set(skeleton.getJointLCS(Skeleton.RIGHT_SHOULDER));
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
		return distancePerSecondLeftHand;
	}

	public float getDistancePerSecondLeftElbow() {
		return distancePerSecondLeftElbow;
	}

	public float getDistancePerSecondRightHand() {
		return distancePerSecondRightHand;
	}

	public float getDistancePerSecondRightElbow() {
		return distancePerSecondRightElbow;
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

	public int getMaxAngleLeftLowerArmIndex() {
		return maxAngleLeftLowerArmIndex;
	}

	public int getMaxAngleLeftUpperArmIndex() {
		return maxAngleLeftUpperArmIndex;
	}

	public int getMaxAngleRightLowerArmIndex() {
		return maxAngleRightLowerArmIndex;
	}

	public int getMaxAngleRightUpperArmIndex() {
		return maxAngleRightUpperArmIndex;
	}

	public PVector getLeftHandAtMaxAngle() {
		return historyLeftHandLCS.get(maxAngleLeftLowerArmIndex);
	}

	public PVector getLeftElbowAtMaxAngle() {
		return historyLeftElbowLCS.get(maxAngleLeftUpperArmIndex);
	}

	public PVector getRightHandAtMaxAngle() {
		return historyRightHandLCS.get(maxAngleRightLowerArmIndex);
	}

	public PVector getRightElbowAtMaxAngle() {
		return historyRightElbowLCS.get(maxAngleRightUpperArmIndex);
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
	
}
