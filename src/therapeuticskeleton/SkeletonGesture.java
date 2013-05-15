package therapeuticskeleton;

import processing.core.*;

public class SkeletonGesture {

	/** No gesture of the upper body joints is recognized */
	public static final short NO_GESTURE = 0;
	/** A push gesture of the upper body joints is recognized when the hand and elbow joints are pushed forward in a quick movement */
	public static final short PUSH_GESTURE = 1;
	/** Stores the number of available gestures */
	public static final short NUMBER_OF_GESTURES = 2;
	
	private short currentUpperBodyGesture = NO_GESTURE;
	private int frameLastBodyGestureRecognized = -9999;
	private float gestureTolerance = 0.5f;
	private float gestureAngleTolerance = PApplet.radians(20)*gestureTolerance;

	// defining variables for push gesture
	private final int pushGestureMaxFrames = 30; // max update cycles to perform push gesture
	public static int pushGestureStartFrame = -9999;
	
	private Skeleton skeleton = null;
	
	public SkeletonGesture (Skeleton _skeleton) {
		skeleton = _skeleton;
	}
	

	/** Setter for the tolerance with which gesture will be detected. 0..1f.
	 *  @param _gestureTolerance the tolerance between 0..1f. when higher than 1 or lower than 0, default tolerance 0.5f will be set */
	public void setGestureTolerance (float _gestureTolerance) {
		if (_gestureTolerance >= 0f && _gestureTolerance <= 1f) {
			gestureTolerance = _gestureTolerance;
		} else {
			gestureTolerance = 0.5f; // default gesture accuracy
		}
		gestureAngleTolerance = PApplet.radians(40)*gestureTolerance;
	}
	/** Getter for gesture tolerance
	 *  @return the gesture tolerance */
	public float getGestureTolerance () {
		return gestureTolerance;
	}

	/** Upper body gesture is evaluated corresponding to the following gestures. 
	 *  --------------------------------------------<br>
	 *  PUSH_GESTURE: A push gesture of the upper body joints is recognized when the hand and elbow joints are pushed forward in a quick movement <br>
	 *  --------------------------------------------<br>
	 *  @param _maxAge the maximum time in the past the last gesture should have been recognized.
	 *  @return current upper body gesture. short, constants of Skeleton class, NO_GESTURE if no gesture was recognized in the given past update cycles or gesture evaluation is switched off */
	public short getLastUpperBodyGesture (int _maxAge) {
		if (frameLastBodyGestureRecognized >= _maxAge)
			return currentUpperBodyGesture;
		else
			return NO_GESTURE;
	}
	
	/** Evaluate gestures and store results internally. Access recognized gestures using getter-methods. 
	 *  @param _frameCount The current update cycle of the main applet. Is used to evaluate time critical gestures. 
	 * */
	public void evaluate (int _frameCount) {
		if (evaluatePushGesture(_frameCount)) {
			currentUpperBodyGesture = PUSH_GESTURE;
			frameLastBodyGestureRecognized = _frameCount;
		} 
	}
	
	// helper functions
	private boolean evaluatePushGesture (int _frameCount) {
		// push gesture is found when a movement of the hands parallel to the body z-axis has occured and the end pose is reached. 
		// The end pose is recognized when upper and lower arms form a straight line parallel to body z-axis. 
		PVector rHandShoulder = PVector.sub(skeleton.getJointLCS(Skeleton.RIGHT_SHOULDER),skeleton.getJointLCS(Skeleton.RIGHT_HAND));
		PVector lHandShoulder = PVector.sub(skeleton.getJointLCS(Skeleton.LEFT_SHOULDER),skeleton.getJointLCS(Skeleton.LEFT_HAND));
		float angleRtoBodyZ = PVector.angleBetween(rHandShoulder, skeleton.getOrientationZ());
		float angleLtoBodyZ = PVector.angleBetween(lHandShoulder, skeleton.getOrientationZ());
		if (SkeletonMath.isValueBetween(angleRtoBodyZ,0,PApplet.radians(30)+gestureAngleTolerance) && SkeletonMath.isValueBetween(angleLtoBodyZ,0,PApplet.radians(30)+gestureAngleTolerance)) {
			float handShoulderDistanceStartPose = 200f+200f*gestureTolerance;
			if (rHandShoulder.mag() <= handShoulderDistanceStartPose && lHandShoulder.mag() <= handShoulderDistanceStartPose) {
				// start pose recognized
				pushGestureStartFrame = _frameCount;
			}
			if (_frameCount-pushGestureStartFrame <= pushGestureMaxFrames) {
				float angleRightArm = PVector.angleBetween(skeleton.getRightUpperArmLCS(),skeleton.getRightLowerArmLCS());
				float angleLeftArm = PVector.angleBetween(skeleton.getLeftUpperArmLCS(),skeleton.getLeftLowerArmLCS());
				if (SkeletonMath.isValueBetween(angleRightArm,0,PApplet.radians(30)+gestureAngleTolerance) && SkeletonMath.isValueBetween(angleLeftArm,0,PApplet.radians(30)+gestureAngleTolerance)) {
					// arms form a straight line
					float angleRightArmToBodyZ = PVector.angleBetween(skeleton.getRightUpperArmLCS(),skeleton.getOrientationZ());
					float angleLeftArmToBodyZ = PVector.angleBetween(skeleton.getLeftUpperArmLCS(),skeleton.getOrientationZ());
					if (SkeletonMath.isValueBetween(angleRightArmToBodyZ,PApplet.radians(150)-gestureAngleTolerance,PApplet.radians(180)) && SkeletonMath.isValueBetween(angleLeftArmToBodyZ,PApplet.radians(150)-gestureAngleTolerance,PApplet.radians(180))) {
						// arms are parallel to body z axis: end pose recognized
						return true;
					}
				}
			} else {
				// didn't recognize end pose within max cycles
				pushGestureStartFrame = -9999;
			}
		} else {
			// the movement needs to be parallel to z axis all the time
			pushGestureStartFrame = -9999;
		}
		return false;
	}
}
