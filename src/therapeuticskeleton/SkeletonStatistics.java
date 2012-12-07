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
			PVector tempLeftHand = new PVector();
			tempLeftHand.set(skeleton.getJointLCS(Skeleton.LEFT_HAND));
			PVector tempLeftElbow = new PVector();
			tempLeftElbow.set(skeleton.getJointLCS(Skeleton.LEFT_ELBOW));
			PVector tempRightHand = new PVector();
			tempRightHand.set(skeleton.getJointLCS(Skeleton.RIGHT_HAND));
			PVector tempRightElbow = new PVector();
			tempRightElbow.set(skeleton.getJointLCS(Skeleton.RIGHT_ELBOW));
			
			distanceLeftHand += skeleton.getJointDelta(Skeleton.LEFT_HAND);
			distanceLeftElbow += skeleton.getJointDelta(Skeleton.LEFT_ELBOW);
			distanceRightHand += skeleton.getJointDelta(Skeleton.RIGHT_HAND);
			distanceRightElbow += skeleton.getJointDelta(Skeleton.RIGHT_ELBOW);
			distancePerSecondLeftHand = skeleton.getJointDelta(Skeleton.LEFT_HAND)*skeleton.getFrameRate();
			distancePerSecondLeftElbow = skeleton.getJointDelta(Skeleton.LEFT_ELBOW)*skeleton.getFrameRate();
			distancePerSecondRightHand = skeleton.getJointDelta(Skeleton.RIGHT_HAND)*skeleton.getFrameRate();
			distancePerSecondRightElbow = skeleton.getJointDelta(Skeleton.RIGHT_ELBOW)*skeleton.getFrameRate();
	
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
			
			historyLeftHandLCS.add(tempLeftHand);
			historyLeftElbowLCS.add(tempLeftElbow);
			historyRightHandLCS.add(tempRightHand);
			historyRightElbowLCS.add(tempRightElbow);
			
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
	
}
