package therapeuticskeleton;

import java.util.ArrayList;
import processing.core.*;

public class SkeletonStatistics {
	private ArrayList<PVector> historyLeftHand = new ArrayList<PVector>();
	private ArrayList<PVector> historyLeftElbow = new ArrayList<PVector>();
	private ArrayList<PVector> historyRightHand = new ArrayList<PVector>();
	private ArrayList<PVector> historyRightElbow = new ArrayList<PVector>();
	
	private float distanceLeftHand = 0f;
	private float distanceLeftElbow = 0f;
	private float distanceRightHand = 0f;
	private float distanceRightElbow = 0f;
	
	private float distancePerSecondLeftHand = 0f;
	private float distancePerSecondLeftElbow = 0f;
	private float distancePerSecondRightHand = 0f;
	private float distancePerSecondRightElbow = 0f;

	private float maxXLeftHand = 0f;
	private float maxYLeftHand = 0f;
	private float maxZLeftHand = 0f;
	private float minXLeftHand = 0f;
	private float minYLeftHand = 0f;
	private float minZLeftHand = 0f;
	private float maxXLeftElbow = 0f;
	private float maxYLeftElbow = 0f;
	private float maxZLeftElbow = 0f;
	private float minXLeftElbow = 0f;
	private float minYLeftElbow = 0f;
	private float minZLeftElbow = 0f;
	private float maxXRightHand = 0f;
	private float maxYRightHand = 0f;
	private float maxZRightHand = 0f;
	private float minXRightHand = 0f;
	private float minYRightHand = 0f;
	private float minZRightHand = 0f;
	private float maxXRightElbow = 0f;
	private float maxYRightElbow = 0f;
	private float maxZRightElbow = 0f;
	private float minXRightElbow = 0f;
	private float minYRightElbow = 0f;
	private float minZRightElbow = 0f;
	
	private Skeleton skeleton = null;
	
	public SkeletonStatistics (Skeleton _skeleton) {
		skeleton = _skeleton;
	}
	
	public void update () {
		PVector tempLeftHand = skeleton.getJoint(Skeleton.LEFT_HAND);
		PVector tempLeftElbow = skeleton.getJoint(Skeleton.LEFT_ELBOW);
		PVector tempRightHand = skeleton.getJoint(Skeleton.RIGHT_HAND);
		PVector tempRightElbow = skeleton.getJoint(Skeleton.RIGHT_ELBOW);
		
		if (historyLeftHand.size() > 0) {
			PVector prevLeftHand = historyLeftHand.get(historyLeftHand.size()-1);
			PVector prevLeftElbow = historyLeftElbow.get(historyLeftElbow.size()-1);
			PVector prevRightHand = historyRightHand.get(historyRightHand.size()-1);
			PVector prevRightElbow = historyRightElbow.get(historyRightElbow.size()-1);
			float currentDistanceLeftHand = (PVector.sub(tempLeftHand,prevLeftHand)).mag();
			float currentDistanceLeftElbow = (PVector.sub(tempLeftElbow,prevLeftElbow)).mag();
			float currentDistanceRightHand = (PVector.sub(tempRightHand,prevRightHand)).mag();
			float currentDistanceRightElbow = (PVector.sub(tempRightElbow,prevRightElbow)).mag();
			distanceLeftHand += currentDistanceLeftHand;
			distanceLeftElbow += currentDistanceLeftElbow;
			distanceRightHand += currentDistanceRightHand;
			distanceRightElbow += currentDistanceRightElbow;
			distancePerSecondLeftHand = currentDistanceLeftHand/skeleton.getFrameRate();
			distancePerSecondLeftElbow = currentDistanceLeftElbow/skeleton.getFrameRate();
			distancePerSecondRightHand = currentDistanceRightHand/skeleton.getFrameRate();
			distancePerSecondRightElbow = currentDistanceRightElbow/skeleton.getFrameRate();
			
		}

		if (tempLeftHand.x > maxXLeftHand) maxXLeftHand = tempLeftHand.x;
		if (tempLeftHand.y > maxYLeftHand) maxYLeftHand = tempLeftHand.y;
		if (tempLeftHand.z > maxZLeftHand) maxZLeftHand = tempLeftHand.z;
		if (tempLeftHand.x < minXLeftHand) minXLeftHand = tempLeftHand.x;
		if (tempLeftHand.y < minYLeftHand) minYLeftHand = tempLeftHand.y;
		if (tempLeftHand.z < minZLeftHand) minZLeftHand = tempLeftHand.z;
		
		if (tempLeftElbow.x > maxXLeftElbow) maxXLeftElbow = tempLeftElbow.x;
		if (tempLeftElbow.y > maxYLeftElbow) maxYLeftElbow = tempLeftElbow.y;
		if (tempLeftElbow.z > maxZLeftElbow) maxZLeftElbow = tempLeftElbow.z;
		if (tempLeftElbow.x < minXLeftElbow) minXLeftElbow = tempLeftElbow.x;
		if (tempLeftElbow.y < minYLeftElbow) minYLeftElbow = tempLeftElbow.y;
		if (tempLeftElbow.z < minZLeftElbow) minZLeftElbow = tempLeftElbow.z;

		if (tempRightHand.x > maxXRightHand) maxXRightHand = tempRightHand.x;
		if (tempRightHand.y > maxYRightHand) maxYRightHand = tempRightHand.y;
		if (tempRightHand.z > maxZRightHand) maxZRightHand = tempRightHand.z;
		if (tempRightHand.x < minXRightHand) minXRightHand = tempRightHand.x;
		if (tempRightHand.y < minYRightHand) minYRightHand = tempRightHand.y;
		if (tempRightHand.z < minZRightHand) minZRightHand = tempRightHand.z;

		if (tempRightElbow.x > maxXRightElbow) maxXRightElbow = tempRightElbow.x;
		if (tempRightElbow.y > maxYRightElbow) maxYRightElbow = tempRightElbow.y;
		if (tempRightElbow.z > maxZRightElbow) maxZRightElbow = tempRightElbow.z;
		if (tempRightElbow.x < minXRightElbow) minXRightElbow = tempRightElbow.x;
		if (tempRightElbow.y < minYRightElbow) minYRightElbow = tempRightElbow.y;
		if (tempRightElbow.z < minZRightElbow) minZRightElbow = tempRightElbow.z;
		
		historyLeftHand.add(tempLeftHand);
		historyLeftElbow.add(tempLeftElbow);
		historyRightHand.add(tempRightHand);
		historyRightElbow.add(tempRightElbow);
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

	public float getMaxXLeftHand() {
		return maxXLeftHand;
	}

	public float getMaxYLeftHand() {
		return maxYLeftHand;
	}

	public float getMaxZLeftHand() {
		return maxZLeftHand;
	}

	public float getMinXLeftHand() {
		return minXLeftHand;
	}

	public float getMinYLeftHand() {
		return minYLeftHand;
	}

	public float getMinZLeftHand() {
		return minZLeftHand;
	}

	public float getMaxXLeftElbow() {
		return maxXLeftElbow;
	}

	public float getMaxYLeftElbow() {
		return maxYLeftElbow;
	}

	public float getMaxZLeftElbow() {
		return maxZLeftElbow;
	}

	public float getMinXLeftElbow() {
		return minXLeftElbow;
	}

	public float getMinYLeftElbow() {
		return minYLeftElbow;
	}

	public float getMinZLeftElbow() {
		return minZLeftElbow;
	}

	public float getMaxXRightHand() {
		return maxXRightHand;
	}

	public float getMaxYRightHand() {
		return maxYRightHand;
	}

	public float getMaxZRightHand() {
		return maxZRightHand;
	}

	public float getMinXRightHand() {
		return minXRightHand;
	}

	public float getMinYRightHand() {
		return minYRightHand;
	}

	public float getMinZRightHand() {
		return minZRightHand;
	}

	public float getMaxXRightElbow() {
		return maxXRightElbow;
	}

	public float getMaxYRightElbow() {
		return maxYRightElbow;
	}

	public float getMaxZRightElbow() {
		return maxZRightElbow;
	}

	public float getMinXRightElbow() {
		return minXRightElbow;
	}

	public float getMinYRightElbow() {
		return minYRightElbow;
	}

	public float getMinZRightElbow() {
		return minZRightElbow;
	}
	
}
