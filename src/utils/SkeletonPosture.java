package utils;

import processing.core.PApplet;
import processing.core.PVector;
import therapeuticskeleton.Skeleton;

public class SkeletonPosture {
	/** Upper body joints form no articulated pose */
	public static final short NO_POSE = 0;
	/** Upper body joints form articulated V shape on the local x-axis: (H = hand, E = elbow, S = shoulder) <br>
	 *  on the x axis:------<br>
	 *  H        H <br>
	 *    E    E			= V <br>
	 *      SS <br>
	 *  ---------------------- */
	public static final short V_SHAPE = 1;
	/** Upper body joints form articulated A shape on the local x-axis: (H = hand, E = elbow, S = shoulder)<br>
	 *  on the x axis:------<br>
	 *      SS<br>
	 *    E    E			= A<br>
	 *  H        H<br>
	 *  ---------------------- */
	public static final short A_SHAPE = 2;
	/** Upper body joints form articulated U shape on the local x-axis: (H = hand, E = elbow, S = shoulder)<br>
	 *  on the x axis:------<br>
	 *  H         H<br>
	 *          			= U<br>
	 *  E  SS  E<br>
	 *  ---------------------- */
	public static final short U_SHAPE = 3;
	/** Upper body joints form articulated N shape on the local x-axis: (H = hand, E = elbow, S = shoulder)<br>
	 *  on the x axis:------<br>
	 *  E  SS  E<br>
	 *  					= N<br>
	 *  H         H<br>
	 *  ---------------------- */
	public static final short N_SHAPE = 4;
	/** Upper body joints form articulated M shape on the local x-axis: (H = hand, E = elbow, S = shoulder)<br>
	 *  on the x axis:------<br>
	 *  E         E<br>
	 *      SS  			= M<br>
	 *  H         H<br>
	 *  ---------------------- */
	public static final short M_SHAPE = 5;
	/** Upper body joints form articulated W shape on the local x-axis: (H = hand, E = elbow, S = shoulder)<br>
	 *  on the x axis:------<br>
	 *  H         H<br>
	 *      SS   			= W<br>
	 *  E         E<br>
	 *  ---------------------- */
	public static final short W_SHAPE = 6;
	/** Upper body joints form articulated O shape on the local x-axis: (H = hand, E = elbow, S = shoulder)<br>
	 *  on the x axis:----------------------------<br>
	 *      HH						        SS<br>
	 *  E         E		= O = 		E        E<br>
	 *      SS						        HH<br>
	 *  -------------------------------------------- */
	public static final short O_SHAPE = 7;
	/** Upper body joints form articulated I shape: (H = hand, E = elbow, S = shoulder)<br>
	 *  on the x axis:----------------------------<br>
	 *      HH							    SS<br>
	 *      EE			= I =			    EE<br>
	 *      SS							    HH<br>
	 *  -------------------------------------------- */
	public static final short I_SHAPE = 8;
	/** Upper body joints form a pose holding the arms straight and the hands down to the level of the hips: (H = hand, E = elbow, S = shoulder)<br>
	 *  on the z axis:----------------------------<br>
	 *  SS<br>
	 *      EE			= HANDS_FORWARD_DOWN<br>
	 *          HH<br>
	 *  -------------------------------------------- */
	public static final short HANDS_FORWARD_DOWN_POSE = 9;
	/** Stores the number of available poses */
	public static final short NUMBER_OF_POSES = 10;
	
	// current upper body posture and gesture
	private short currentUpperBodyPosture = NO_POSE;
	
	private float postureTolerance = 0.3f;
	private float postureAngleTolerance = PApplet.radians(20)*postureTolerance;

	private Skeleton skeleton = null;
	
	public SkeletonPosture (Skeleton _skeleton) {
		skeleton = _skeleton;
	}

	/** Setter for the tolerance with which posture will be detected. 0..1f.
	 *  @param the tolerance between 0..1f. when higher than 1 or lower than 0, default tolerance 0.3f will be set */
	public void setPostureTolerance (float _postureTolerance) {
		if (_postureTolerance >= 0f && _postureTolerance <= 1f) {
			postureTolerance = _postureTolerance;
		} else {
			postureTolerance = 0.3f; // default posture accuracy
		}
		postureAngleTolerance = PApplet.radians(20)*postureTolerance;
	}
	/** Getter for posture tolerance
	 *  @return the posture tolerance */
	public float getPostureTolerance () {
		return postureTolerance;
	}
	/** Upper body posture is evaluated corresponding to one of the following shapes on the local x-axis or to one of the other articulated poses. 
	 *  This method returns the evaluated upper body posture. works only if posture was calculated in the current update cycle<p>
	 *  (H = hand, E = elbow, S = shoulder)<br>
	 *  ----------------------<br>
	 *  H        H<br>
	 *    E    E			= V<br>
	 *      SS<br>
	 *  ----------------------<br>
	 *      SS<br>
	 *    E    E			= A<br>
	 *  H        H<br>
	 *  ----------------------<br>
	 *  H         H<br>
	 *          			= U<br>
	 *  E  SS  E<br>
	 *  ----------------------<br>
	 *  E  SS  E<br>
	 *  					= N<br>
	 *  H         H<br>
	 *  ----------------------<br>
	 *  E         E<br>
	 *      SS  			= M<br>
	 *  H         H<br>
	 *  ----------------------<br>
	 *  H         H<br>
	 *      SS   			= W<br>
	 *  E         E<br>
	 *  --------------------------------------------<br>
	 *      HH						        SS<br>
	 *  E         E		= O = 		E        E<br>
	 *      SS						        HH<br>
	 *  --------------------------------------------<br>
	 *      HH							    SS<br>
	 *      EE			= I =			    EE<br>
	 *      SS							    HH<br>
	 *  --------------------------------------------<br>
	 *  on the z axis:----------------------------<br>
	 *  SS<br>
	 *      EE			= HANDS_FORWARD_DOWN<br>
	 *          HH<br>
	 *  --------------------------------------------
	 *  @return current upper body posture. short, constants of Skeleton class, NO_POSE if posture was not updated */
	public short getCurrentUpperBodyPosture() {
		return currentUpperBodyPosture; 
	}
	
	/** Evaluate posture and store results internally. Access recognized posture using getter-methods. */
	public void evaluate () {
		if (evaluateVShape()) currentUpperBodyPosture = V_SHAPE;
		else if (evaluateAShape()) currentUpperBodyPosture = A_SHAPE;
		else if (evaluateUShape()) currentUpperBodyPosture = U_SHAPE;
		else if (evaluateNShape()) currentUpperBodyPosture = N_SHAPE;
		else if (evaluateMShape()) currentUpperBodyPosture = M_SHAPE;
		else if (evaluateWShape()) currentUpperBodyPosture = W_SHAPE;
		else if (evaluateOShape()) currentUpperBodyPosture = O_SHAPE;
		else if (evaluateIShape()) currentUpperBodyPosture = I_SHAPE;
		else if (evaluateHandsForwardDownPose()) currentUpperBodyPosture = HANDS_FORWARD_DOWN_POSE;
		else currentUpperBodyPosture = NO_POSE;
	}
	
	private boolean evaluateIShape() {
		float angleLArm = PVector.angleBetween(skeleton.getLeftUpperArmLocal(),skeleton.getLeftLowerArmLocal());
		float angleRArm = PVector.angleBetween(skeleton.getRightUpperArmLocal(),skeleton.getRightLowerArmLocal());
		float angleIShape = PVector.angleBetween(skeleton.getLeftUpperArmLocal(),skeleton.getRightUpperArmLocal());
		float angleToBodyY = PVector.angleBetween(skeleton.getLeftUpperArmLocal(),skeleton.getOrientationY());
		if (SkeletonMath.isValueBetween(angleLArm,0,PApplet.radians(10)+postureAngleTolerance) && SkeletonMath.isValueBetween(angleRArm,0,PApplet.radians(10)+postureAngleTolerance)) { // arms form a straight line
			if (SkeletonMath.isValueBetween(angleIShape,0,PApplet.radians(15)+postureAngleTolerance)) { // arms are parallel
				if (SkeletonMath.isValueBetween(angleToBodyY,0,PApplet.radians(15)+postureAngleTolerance)) { // arms are parallel to y body axis
					return true;
				}
			}
		}
		return false;
	}
	private boolean evaluateOShape() {
		PVector rHandlHand = PVector.sub(skeleton.getJointLocalCoordSys(Skeleton.RIGHT_HAND),skeleton.getJointLocalCoordSys(Skeleton.LEFT_HAND));
		if (SkeletonMath.isValueBetween(rHandlHand.mag(),0,100+(100*postureTolerance))) {
			float angleLUpper = PVector.angleBetween(skeleton.getLeftUpperArmLocal(),skeleton.getOrientationY());
			float angleRUpper = PVector.angleBetween(skeleton.getRightUpperArmLocal(),skeleton.getOrientationY());
			float angleLLower = PVector.angleBetween(skeleton.getLeftLowerArmLocal(),skeleton.getLeftUpperArmLocal());
			float angleRLower = PVector.angleBetween(skeleton.getRightLowerArmLocal(),skeleton.getRightUpperArmLocal());
			float angleToBody = PVector.angleBetween(PVector.add(skeleton.getLeftUpperArmLocal(),skeleton.getRightUpperArmLocal()),skeleton.getOrientationY());
			if (SkeletonMath.isValueBetween(angleLUpper,PApplet.radians(40)-postureAngleTolerance,PApplet.radians(50)+postureAngleTolerance) && SkeletonMath.isValueBetween(angleRUpper,PApplet.radians(40)-postureAngleTolerance,PApplet.radians(50)+postureAngleTolerance)) { // ~45 degree
				if (SkeletonMath.isValueBetween(angleLLower,PApplet.radians(95)-postureAngleTolerance,PApplet.radians(105)+postureAngleTolerance) && SkeletonMath.isValueBetween(angleRLower,PApplet.radians(95)-postureAngleTolerance,PApplet.radians(105)+postureAngleTolerance)) { // ~100 degree
					if (SkeletonMath.isValueBetween(angleToBody,0,PApplet.radians(15)+postureAngleTolerance)) { // sum of upper arms parallel to body y axis
						return true;
					}
				}
			}
		}
		return false;
	}
	private boolean evaluateWShape() {
		// TODO Auto-generated method stub
		return false;
	}
	private boolean evaluateMShape() {
		// TODO Auto-generated method stub
		return false;
	}
	private boolean evaluateNShape() {
		float angleL = PVector.angleBetween(skeleton.getLeftUpperArmLocal(),skeleton.getLeftLowerArmLocal());
		float angleR = PVector.angleBetween(skeleton.getRightUpperArmLocal(),skeleton.getRightLowerArmLocal());
		float angleNShape = PVector.angleBetween(skeleton.getLeftUpperArmLocal(), skeleton.getRightUpperArmLocal());
		float angleToBodyY = PVector.angleBetween(skeleton.getLeftLowerArmLocal(),skeleton.getOrientationY());
		if (SkeletonMath.isValueBetween(angleL,PApplet.radians(85)-postureAngleTolerance,PApplet.radians(95)+postureAngleTolerance) && SkeletonMath.isValueBetween(angleR,PApplet.radians(85)-postureAngleTolerance,PApplet.radians(95)+postureAngleTolerance)) { // arms angle ~90 degree
			if (SkeletonMath.isValueBetween(angleNShape,PApplet.radians(170)-postureAngleTolerance,PApplet.radians(180))) { // upper arms form a straight line
				if (SkeletonMath.isValueBetween(angleToBodyY,PApplet.radians(165)-postureAngleTolerance,PApplet.radians(180))) {// arms downwards 
					return true;
				}
			}
		}
		return false;
	}
	private boolean evaluateUShape() {
		float angleL = PVector.angleBetween(skeleton.getLeftUpperArmLocal(),skeleton.getLeftLowerArmLocal());
		float angleR = PVector.angleBetween(skeleton.getRightUpperArmLocal(),skeleton.getRightLowerArmLocal());
		float angleUShape = PVector.angleBetween(skeleton.getLeftUpperArmLocal(), skeleton.getRightUpperArmLocal());
		float angleToBodyY = PVector.angleBetween(skeleton.getLeftLowerArmLocal(),skeleton.getOrientationY());
		if (SkeletonMath.isValueBetween(angleL,PApplet.radians(85)-postureAngleTolerance,PApplet.radians(95)+postureAngleTolerance) && SkeletonMath.isValueBetween(angleR,PApplet.radians(85)-postureAngleTolerance,PApplet.radians(95)+postureAngleTolerance)) { // arms angle ~90 degree
			if (SkeletonMath.isValueBetween(angleUShape,PApplet.radians(170)-postureAngleTolerance,PApplet.radians(180))) { // upper arms form a straight line
				if (SkeletonMath.isValueBetween(angleToBodyY,0,PApplet.radians(15)+postureAngleTolerance)) {// arms upwards 
					return true;
				}
			}
		}
		return false;
	}
	private boolean evaluateAShape() {
		float angleL = PVector.angleBetween(skeleton.getLeftUpperArmLocal(),skeleton.getLeftLowerArmLocal());
		float angleR = PVector.angleBetween(skeleton.getRightUpperArmLocal(),skeleton.getLeftLowerArmLocal());
		float angleAShape = PVector.angleBetween(skeleton.getLeftUpperArmLocal(), skeleton.getRightUpperArmLocal());
		float angleToBody = PVector.angleBetween(PVector.add(skeleton.getLeftUpperArmLocal(),skeleton.getRightUpperArmLocal()),skeleton.getOrientationY());
		if (SkeletonMath.isValueBetween(angleL,0,PApplet.radians(10)+postureAngleTolerance) && SkeletonMath.isValueBetween(angleR,0,PApplet.radians(10)+postureAngleTolerance)) { // arms form a straight line
			if (SkeletonMath.isValueBetween(angleAShape,PApplet.radians(85)-postureAngleTolerance,PApplet.radians(95)+postureAngleTolerance)) { // arms angle ~90 degree
				if (SkeletonMath.isValueBetween(angleToBody,PApplet.radians(165)-postureAngleTolerance,PApplet.radians(180))) { // sum of upper arms parallel to body y axis
					return true;
				}
			}
		}
		return false;
	}
	private boolean evaluateVShape() {
		float angleL = PVector.angleBetween(skeleton.getLeftUpperArmLocal(),skeleton.getLeftLowerArmLocal());
		float angleR = PVector.angleBetween(skeleton.getRightUpperArmLocal(),skeleton.getRightLowerArmLocal());
		float angleVShape = PVector.angleBetween(skeleton.getLeftUpperArmLocal(), skeleton.getRightUpperArmLocal());
		float angleToBody = PVector.angleBetween(PVector.add(skeleton.getLeftUpperArmLocal(),skeleton.getRightUpperArmLocal()),skeleton.getOrientationY());
		if (SkeletonMath.isValueBetween(angleL,0,PApplet.radians(10)+postureAngleTolerance) && SkeletonMath.isValueBetween(angleR,0,PApplet.radians(10)+postureAngleTolerance)) { // arms form a straight line
			if (SkeletonMath.isValueBetween(angleVShape,PApplet.radians(85)-postureAngleTolerance,PApplet.radians(95)+postureAngleTolerance)) { // arms angle ~90 degree
				if (SkeletonMath.isValueBetween(angleToBody,0,PApplet.radians(15)+postureAngleTolerance)) { // sum of upper arms parallel to body y axis
					return true;
				}
			}
		}
		return false;
	}
	private boolean evaluateHandsForwardDownPose() {
		float angleL = PVector.angleBetween(skeleton.getLeftUpperArmLocal(),skeleton.getLeftLowerArmLocal());
		float angleR = PVector.angleBetween(skeleton.getRightUpperArmLocal(),skeleton.getRightLowerArmLocal());
		float angleUpperArms = PVector.angleBetween(skeleton.getLeftUpperArmLocal(), skeleton.getRightUpperArmLocal());
		float angleDownward = PVector.angleBetween(skeleton.getLeftUpperArmLocal(),skeleton.getOrientationY());
		float angleForward = PVector.angleBetween(skeleton.getLeftUpperArmLocal(),skeleton.getOrientationZ());
		if (SkeletonMath.isValueBetween(angleL,0,PApplet.radians(10)+postureAngleTolerance) && SkeletonMath.isValueBetween(angleR,0,PApplet.radians(10)+postureAngleTolerance)) { // arms form a straight line
			if (SkeletonMath.isValueBetween(angleUpperArms,0,PApplet.radians(15)+postureAngleTolerance)) { // arms are parallel
				if (SkeletonMath.isValueBetween(angleDownward,PApplet.radians(130)-postureAngleTolerance,PApplet.radians(140)+postureAngleTolerance)) {// arms downward 45 degree 
					if (SkeletonMath.isValueBetween(angleForward,PApplet.radians(90),PApplet.radians(180))) {// arms forward 
						return true;
					}
				}
			}
		}
		return false;
	}
}
