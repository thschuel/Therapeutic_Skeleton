package therapeuticskeleton;

import SimpleOpenNI.SimpleOpenNI;
import processing.core.*;

public class Skeleton {

	// mirror therapy modes
	public static final short MIRROR_THERAPY_OFF = 0; 
	public static final short MIRROR_THERAPY_LEFT = 1;
	public static final short MIRROR_THERAPY_RIGHT = 2;
	
	// available skeleton joints
	public static final short HEAD = 0; 
	public static final short NECK = 1;
	public static final short LEFT_SHOULDER = 2;
	public static final short LEFT_ELBOW = 3;
	public static final short LEFT_HAND = 4;
	public static final short RIGHT_SHOULDER = 5;
	public static final short RIGHT_ELBOW = 6;
	public static final short RIGHT_HAND = 7;
	public static final short TORSO = 8;
	public static final short LEFT_HIP = 9;
	public static final short LEFT_KNEE = 10;
	public static final short LEFT_FOOT = 11;
	public static final short RIGHT_HIP = 12;
	public static final short RIGHT_KNEE = 13;
	public static final short RIGHT_FOOT = 14;
	
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
	
	// current upper body posture
	private short currentUpperBodyPosture = NO_POSE;
	private boolean postureEvaluated = false;
	private float postureTolerance = 0.3f;
	// The interface to talk to kinect
	private SimpleOpenNI kinect;
	// stores skeleton Points in 3d Space, global coordsys
	private PVector[] joints = new PVector[15]; 
	private float[] confidenceJoints = new float[15];
	// for convenience store vectors of upper arms and lower arms
	private PVector lUpperArm = new PVector();
	private PVector lLowerArm = new PVector();
	private PVector rUpperArm = new PVector();
	private PVector rLowerArm = new PVector();
	private PVector lUpperArmLocal = new PVector();
	private PVector lLowerArmLocal = new PVector();
	private PVector rUpperArmLocal = new PVector();
	private PVector rLowerArmLocal = new PVector();
	// stores skeleton Points in 3d Space, local coordsys (neck is origin)
	private PVector[] jointsLocal = new PVector[15]; 
	private PVector origin;
	private PVector orientationX, orientationY, orientationZ;
	private PMatrix3D transformCoordSys;
	private PMatrix3D transformCoordSysInv;
	// stores joint orientation
	private PMatrix3D[] jointOrientations = new PMatrix3D[15];
	private float[] confidenceJointOrientations = new float[15];
	// calculation of mirror plane
	private PVector[] bodyPoints = new PVector[7]; // stores body points of skeleton 
	private PVector	rMP = new PVector(); // MirrorPlane in HNF: r*n0-d=0
	private PVector	n0MP = new PVector();
	private float dMP = 0.0f;
	// setup variables
	private boolean calculateLocalCoordSys = true;
	private boolean fullBodyTracking = true;
	private short mirrorTherapy = MIRROR_THERAPY_OFF;
	// controls state of skeleton
	private boolean isUpdated = false;
	private boolean mirrorPlaneCalculated = false;
	private boolean localCoordSysCalculated = false;
	// skeleton of user
	private int userId;
	
	// -----------------------------------------------------------------
	// CONSTRUCTORS AND STATECONTROL
	/** Constructor for the Skeleton.
	 *  @param _kinect Handle to the SimpleOpenNI object. Skeleton will maintain its status in the update method by talking to SimpleOpenNI directly.
	 *  @param _userId the user ID of the skeleton
	 *  @param _fullBodyTracking switches full body tracking on/off. If switched off, only upper body joints will be evaluated
	 *  @param _calculateLocalCoordSys switches calculation of the local coordinate system on/off. If switched on, local coordination system will be calculated and joints will be transformed to it 
	 *  @param _mirrorTherapy Sets the skeleton to mirror one body side to the other. When mirrorTherapy is set on, mirrorPlane will be calculated. Short value should correspond to skeleton constants. If out of range, mirror therapy will be switched off */
	public Skeleton (SimpleOpenNI _kinect, int _userId, boolean _fullBodyTracking, boolean _calculateLocalCoordSys, short _mirrorTherapy) {
		kinect = _kinect;
		userId = _userId;
		fullBodyTracking = _fullBodyTracking;
		calculateLocalCoordSys = _calculateLocalCoordSys;
		if (_mirrorTherapy >= MIRROR_THERAPY_OFF && _mirrorTherapy <= MIRROR_THERAPY_RIGHT) 
			mirrorTherapy = _mirrorTherapy;
		for (int i=0; i<15; i++){
			joints[i] = new PVector();
			jointsLocal[i] = new PVector();
			jointOrientations[i] = new PMatrix3D();
		}
		for (int i=0; i<7; i++){
			bodyPoints[i] = new PVector();
		}
	}
	/** Constructor for the Skeleton. Defaults used for setup.
	 *  @param _kinect Handle to the SimpleOpenNI object. Skeleton will maintain its status in the update method by talking to SimpleOpenNI directly.
	 *  @param _userId the user ID of the skeleton */
	public Skeleton (SimpleOpenNI _kinect, int _userId) {
		kinect = _kinect;
		userId = _userId;
		for (int i=0; i<15; i++){
			joints[i] = new PVector();
			jointsLocal[i] = new PVector();
			jointOrientations[i] = new PMatrix3D();
		}
		for (int i=0; i<7; i++){
			bodyPoints[i] = new PVector();
		}
	}

	/** Update method. Call it to update status of skeleton. Skeleton will talk to SimpleOpenNI directly and will do all the necessary math for updating its status according to set up */
	public void update () {
		isUpdated = false;
		localCoordSysCalculated = false;
		mirrorPlaneCalculated = false;
		postureEvaluated = false;
		
		updateJointPositions();
		updateJointOrientations();
		
		if (mirrorTherapy != MIRROR_THERAPY_OFF) {
			calculateMirrorPlane();
			updateMirroredJointPositions();
			updateMirroredJointOrientations();
		}
		if (calculateLocalCoordSys) {
			calculateLocalCoordSys();
			transformToLocalCoordSys();
		}
		
		isUpdated = true;
	}
	// -----------------------------------------------------------------
	// GETTERS AND SETTERS
	/** Setter for mirror therapy modus. Sets the skeleton to mirror one body side to the other. When mirrorTherapy is set on, mirrorPlane will be calculated
	 *  @param _mirrorTherapy short corresponding to Skeleton constants. If out of range, mirror therapy mode will be switched off */
	public void setMirrorTherapy (short _mirrorTherapy) {
		if (_mirrorTherapy >= MIRROR_THERAPY_OFF && _mirrorTherapy <= MIRROR_THERAPY_RIGHT) 
			mirrorTherapy = _mirrorTherapy;
		else
			mirrorTherapy = MIRROR_THERAPY_OFF;
	}
	/** Getter for mirror therapy modus. 
	 *  @return short mirror therapy modus corresponding to Skeleton constants */
	public short getMirrorTherapy () {
		return mirrorTherapy;
	}
	/** Setter for fullBodyTracking. If full body tracking is switched off, only upper body joints will be evaluated.
	 *  @param _fullBodyTracking switch full body tracking on/off */
	public void setFullBodyTracking (boolean _fullBodyTracking) {
		fullBodyTracking = _fullBodyTracking;
	}
	/** Getter for full body tracking. 
	 *  @return true if full body tracking is switched on */
	public boolean getFullBodyTracking () {
		return fullBodyTracking;
	}
	/** Setter for calculating the local coordinate system. If switched on, joints will be transformed to local coordinate system.
	 *  @param _calculateLocalCoordSys switch calculating the local coordinate system on/off */
	public void setCalculateLocalCoordSys (boolean _calculateLocalCoordSys) {
		calculateLocalCoordSys = _calculateLocalCoordSys;
	}
	/** Getter for calculating the local coordinate system.
	 *  @return true if calculating the local coordinate system is switched on */
	public boolean getCalculateLocalCoordSys () {
		return calculateLocalCoordSys;
	}
	/** Setter for the tolerance with which posture will be detected. 0..1f.
	 *  @param the tolerance between 0..1f. when higher than 1 or lower than 0, default tolerance 0.3f will be set */
	public void setPostureTolerance (float _postureTolerance) {
		if (_postureTolerance >= 0f && _postureTolerance <= 1f)
			postureTolerance = _postureTolerance;
		else
			postureTolerance = 0.3f; // default posture accuracy
	}
	/** Getter for calculating the local coordinate system.
	 *  @return true if calculating the local coordinate system is switched on */
	public float getPostureTolerance () {
		return postureTolerance;
	}
	/** Getter for status of the skeleton. Is used as a lock, methods of skeleton will return unsafe values, as long update function is not done.
	 *  @return true if the status of the skeleton is fully updated */
	public boolean isUpdated() {
		return isUpdated;
	}
	/** Getter for user ID of the skeleton.
	 *  @return the user ID of the skeleton */
	public int getUserId() {
		return userId;
	}
	/** This method returns the joint position of a certain joint in the global coordinate system
	 *  @param jointType The joint for which confidence value should be returned. Should be a short value corresponding to Skeleton constants.
	 *  @return The position of a certain joint in the global coordinate system as vector. If jointType out of range: 0-vector */
	public PVector getJoint (short jointType) {
		if (jointType >= 0 && jointType <= 14) 
			return joints[jointType];
		else
			return new PVector();
	}
	/** This method returns the joint position of a certain joint on the kinect's projective plane. Z-value will be 0
	 *  @param jointType The joint for which confidence value should be returned. Should be a short value corresponding to Skeleton constants.
	 *  @return The position of a certain joint as vector on the projective plane of the kinect. If jointType out of range: 0-vector */
	public PVector getJointProjective (short jointType) {
		if (jointType >= 0 && jointType <= 14) {
			PVector projective = new PVector();
			kinect.convertRealWorldToProjective(joints[jointType], projective);
			return projective;
		} else {
			return new PVector();
		}
	}
	/** The positions of the joints are transformed to the local coordinate system of the skeleton if calculateLocalCoordSys was set.
	 *  This method returns the joint position of a certain joint in the local coordinate system. Works only if localCoordSysCalculated is true.
	 *  @param jointType The joint for which confidence value should be returned. Should be a short value corresponding to Skeleton constants.
	 *  @return The position of a certain joint in the local coordinate system as vector. If jointType out of range or if localCoordSys was not calculated: 0-vector */
	public PVector getJointLocalCoordSys (short jointType) {
		if (jointType >= 0 && jointType <= 14 && localCoordSysCalculated) 
			return jointsLocal[jointType];
		else
			return new PVector();
	}
	/** The vectors for lower and upper arms are calculated for convenience. 
	 *  @return The vector for the left upper arm */
	public PVector getLeftUpperArm() {
		return lUpperArm;
	}
	/** The vectors for lower and upper arms are calculated for convenience. 
	 *  @return The vector for the left lower arm */
	public PVector getLeftLowerArm() {
		return lLowerArm;
	}
	/** The vectors for lower and upper arms are calculated for convenience. 
	 *  @return The vector for the right upper arm */
	public PVector getRightUpperArm() {
		return rUpperArm;
	}
	/** The vectors for lower and upper arms are calculated for convenience. 
	 *  @return The vector for the right lower arm */
	public PVector getRightLowerArm() {
		return rLowerArm;
	}
	/** The vectors for the lower and upper arms are calculated for convenience. The arms in the local coordinate system are calculated only if calculateLocalCoordSys was set.
	 *  @return The vector for the left upper arm in the local coordinate system. If localCoordSys was not calculated: 0-vector */
	public PVector getLeftUpperArmLocal() {
		if (localCoordSysCalculated)
			return lUpperArmLocal;
		else
			return new PVector();
	}
	/** The vectors for the lower and upper arms are calculated for convenience. The arms in the local coordinate system are calculated only if calculateLocalCoordSys was set.
	 *  @return The vector for the left lower arm in the local coordinate system. If localCoordSys was not calculated: 0-vector */
	public PVector getRightUpperArmLocal() {
		if (localCoordSysCalculated)
			return lLowerArmLocal;
		else
			return new PVector();
	}
	/** The vectors for the lower and upper arms are calculated for convenience. The arms in the local coordinate system are calculated only if calculateLocalCoordSys was set.
	 *  @return The vector for the right upper arm in the local coordinate system. If localCoordSys was not calculated: 0-vector */
	public PVector getLeftLowerArmLocal() {
		if (localCoordSysCalculated)
			return rUpperArmLocal;
		else
			return new PVector();
	}
	/** The vectors for the lower and upper arms are calculated for convenience. The arms in the local coordinate system are calculated only if calculateLocalCoordSys was set.
	 *  @return The vector for the right lower arm in the local coordinate system. If localCoordSys was not calculated: 0-vector */
	public PVector getRightLowerArmLocal() {
		if (localCoordSysCalculated)
			return rLowerArmLocal;
		else
			return new PVector();
	}
	/** The positions of the joints are evaluated with a certain confidence value. This method returns the confidence value for a certain joint
	 *  @param jointType The joint for which confidence value should be returned. Should be a short value corresponding to Skeleton constants.
	 *  @return The confidence value of a certain joint. Between 0f and 1f. If jointType out of range: 0f */
	public float getConfidenceJoint (short jointType) {
		if (jointType >= 0 && jointType <= 14) 
			return confidenceJointOrientations[jointType];
		else
			return 0f;
	}
	/** The orientations of the joints are evaluated with a certain confidence value. This method returns the orientation matrix 
	 *  @param jointType The joint for which confidence value should be returned. Should be a short value corresponding to Skeleton constants.
	 *  @return The orientation matrix of a certain joint. PMatrix3D. If jointType out of range: 0-Matrix */
	public PMatrix3D getJointOrientation (short jointType) {
		if (jointType >= 0 && jointType <= 14) 
			return jointOrientations[jointType];
		else
			return new PMatrix3D();
	}
	/** The orientations of the joints are evaluated with a certain confidence value. This method returns the confidence value 
	 *  @param jointType The joint for which confidence value should be returned. Should be a short value corresponding to Skeleton constants.
	 *  @return The confidence value for the evaluated orientation of a certain joint. Between 0f and 1f. If jointType out of range: 0f */
	public float getConfidenceJointOrientation (short jointType) {
		if (jointType >= 0 && jointType <= 14) 
			return confidenceJointOrientations[jointType];
		else
			return 0f;
	}
	/** Mirror Plane is defined in HNF: r*n0-d = 0. works only if mirror plane has been calculated, i.e. skeleton is in mirror therapy mode.
	 *  @return the r vector of the mirrorPlane if mirror plane was calculated. Else 0-vector */
	public PVector getRVectorMirrorPlane () {
		if (mirrorPlaneCalculated)
			return rMP;
		else
			return new PVector();
	}
	/** Mirror Plane is defined in HNF: r*n0-d = 0. works only if mirror plane has been calculated, i.e. skeleton is in mirror therapy mode.
	 *  @return the normal vector of the mirrorPlane if mirror plane was calculated. Else 0-vector */
	public PVector getN0VectorMirrorPlane () {
		if (mirrorPlaneCalculated)
			return n0MP;
		else
			return new PVector();
	}
	/** Mirror Plane is defined in HNF: r*n0-d = 0. works only if mirror plane has been calculated, i.e. skeleton is in mirror therapy mode.
	 *  @return the distance of the mirrorPlane to the origin if mirror plane was calculated. Else 0f */
	public float getDValueMirrorPlane () {
		if (mirrorPlaneCalculated)
			return dMP;
		else
			return 0f;
	}
	/** returns the origin of the local coordsys. Equals Torso Vector. works only if localCoordSys has been calculated
	 *  @return the origin. 0-vector if localCoordSys has not been calculated */
	public PVector getOrigin () {
		if (localCoordSysCalculated)
			return origin;
		else
			return new PVector();
	}
	/** returns the local x coordinate vector. works only if localCoordSys has been calculated
	 *  @return the local vector. 0-vector if localCoordSys has not been calculated */
	public PVector getOrientationX () {
		if (localCoordSysCalculated)
			return orientationX;
		else
			return new PVector();
	}
	/** returns the local x coordinate vector projected to the kinects projectionn plane. works only if localCoordSys has been calculated
	 *  @return the local vector projected to the kinects projection plane. 0-vector if localCoordSys has not been calculated */
	public PVector getOrientationXProjective () {
		if (localCoordSysCalculated) {
			PVector projective = new PVector();
			kinect.convertRealWorldToProjective(orientationX,projective);
			return projective;
		} else {
			return new PVector();
		}
	}
	/** returns the angle between the local x vector and the global x vector. works only if localCoordSys has been calculated
	 *  @return the angle, float between 0 and PI. 0f when localCoordSys has not been calculated */
	public float getOrientationAlpha () {
		if (localCoordSysCalculated)
			return PVector.angleBetween(orientationX,new PVector(1,0,0));
		else
			return 0f;
	}
	/** returns the local y coordinate vector. works only if localCoordSys has been calculated
	 *  @return the local vector. 0-vector if localCoordSys has not been calculated */
	public PVector getOrientationY () {
		if (localCoordSysCalculated)
			return orientationY;
		else
			return new PVector();
	}
	/** returns the local y coordinate vector projected to the kinects projectionn plane. works only if localCoordSys has been calculated
	 *  @return the local vector projected to the kinects projection plane. 0-vector if localCoordSys has not been calculated */
	public PVector getOrientationYProjective () {
		if (localCoordSysCalculated) {
			PVector projective = new PVector();
			kinect.convertRealWorldToProjective(orientationY,projective);
			return projective;
		} else {
			return new PVector();
		}
	}
	/** returns the angle between the local y vector and the global y vector. works only if localCoordSys has been calculated
	 *  @return the angle, float between 0 and PI. 0f when localCoordSys has not been calculated */
	public float getOrientationBeta () {
		if (localCoordSysCalculated)
			return PVector.angleBetween(orientationY,new PVector(0,1,0));
		else
			return 0f;
	}
	/** returns the local z coordinate vector. works only if localCoordSys has been calculated
	 *  @return the local vector. 0-vector if localCoordSys has not been calculated */
	public PVector getOrientationZ () {
		if (localCoordSysCalculated)
			return orientationZ;
		else
			return new PVector();
	}
	/** returns the local z coordinate vector projected to the kinects projectionn plane. works only if localCoordSys has been calculated
	 *  @return the local vector projected to the kinects projection plane. 0-vector if localCoordSys has not been calculated */
	public PVector getOrientationZProjective () {
		if (localCoordSysCalculated) {
			PVector projective = new PVector();
			kinect.convertRealWorldToProjective(orientationZ,projective);
			return projective;
		} else {
			return new PVector();
		}
	}
	/** returns the angle between the local z vector and the global z vector. works only if localCoordSys has been calculated
	 *  @return the angle, float between 0 and PI. 0f when localCoordSys has not been calculated*/
	public float getOrientationGamma () {
		if (localCoordSysCalculated) {
			return PVector.angleBetween(orientationZ,new PVector(0,0,1));
		} else
			return 0f;
	}
	/** returns the angle between two limb-vectors
	 *  @param joint11 the joint the limb-vector1 points to
	 *  @param joint12 the joint the limb-vector1 origins in
	 *  @param joint21 the joint the limb-vector2 points to
	 *  @param joint22 the joint the limb-vector2 origins in
	 *  @return the angle, float between 0 and PI */
	public float angleBetween (short joint11, short joint12, short joint21, short joint22) {
		PVector axis1 = PVector.sub(joints[joint11],joints[joint12]);
		PVector axis2 = PVector.sub(joints[joint21],joints[joint22]);
		return PVector.angleBetween(axis1,axis2);
	}
	/** returns the angle between the limb-vector and local X axis
	 *  @param joint11 the joint the limb-vector points to
	 *  @param joint12 the joint the limb-vector origins in
	 *  @return the angle, float between 0 and PI */
	public float angleToLocalXAxis (short joint11, short joint12) {
		PVector axis1 = PVector.sub(jointsLocal[joint11],jointsLocal[joint12]);
		return PVector.angleBetween(axis1,orientationX);
	}
	/** returns the angle between the limb-vector and local Y axis
	 *  @param joint11 the joint the limb-vector points to
	 *  @param joint12 the joint the limb-vector origins in
	 *  @return the angle, float between 0 and PI */
	public float angleToLocalYAxis (short joint11, short joint12) {
		PVector axis1 = PVector.sub(jointsLocal[joint11],jointsLocal[joint12]);
		return PVector.angleBetween(axis1,orientationY);
	}
	/** returns the angle between the limb-vector and local Z axis
	 *  @param joint11 the joint the limb-vector points to
	 *  @param joint12 the joint the limb-vector origins in
	 *  @return the angle, float between 0 and PI */
	public float angleToLocalZAxis (short joint11, short joint12) {
		PVector axis1 = PVector.sub(jointsLocal[joint11],jointsLocal[joint12]);
		return PVector.angleBetween(axis1,orientationZ);
	}
	/** returns the angle between the limb-vector and the global X axis
	 *  @param joint11 the joint the limb-vector points to
	 *  @param joint12 the joint the limb-vector origins in
	 *  @return the angle, float between 0 and PI */
	public float angleToXAxis (short joint11, short joint12) {
		PVector axis1 = PVector.sub(joints[joint11],joints[joint12]);
		return PVector.angleBetween(axis1,new PVector(1,0,0));
	}
	/** returns the angle between the limb-vector and the global Y axis
	 *  @param joint11 the joint the limb-vector points to
	 *  @param joint12 the joint the limb-vector origins in
	 *  @return the angle, float between 0 and PI */
	public float angleToYAxis (short joint11, short joint12) {
		PVector axis1 = PVector.sub(joints[joint11],joints[joint12]);
		return PVector.angleBetween(axis1,new PVector(0,1,0));
	}
	/** returns the angle between the limb-vector and the global Z axis
	 *  @param joint11 the joint the limb-vector points to
	 *  @param joint12 the joint the limb-vector origins in
	 *  @return the angle, float between 0 and PI */
	public float angleToZAxis (short joint11, short joint12) {
		PVector axis1 = PVector.sub(joints[joint11],joints[joint12]);
		return PVector.angleBetween(axis1,new PVector(0,0,1));
	}
	/** returns the distance of the skeletons torso joint to the kinect
	 *  @return the distance in mm, magnitude of skeletons torso vector */
	public float distanceToKinect () {
		return joints[Skeleton.TORSO].mag();
	}
	
	// -----------------------------------------------------------------
	// POSTURE ACCESS
	/** Evaluates if current upper body posture corresponds to one of the following shapes on the local x-axis or to one of the other articulated poses:<p>
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
	 *  @return current upper body posture. short, constants of Skeleton class */
	public short evaluateUpperJointPosture () {
		if (isUpdated && localCoordSysCalculated) {
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
			postureEvaluated = true;
			return currentUpperBodyPosture; 
		} else {
			currentUpperBodyPosture = NO_POSE;
			postureEvaluated = false;
			return currentUpperBodyPosture; 
		}
	}
	/** Returns the evaluated upper body posture. works only if posture was calculated in the current update cycle
	 * @return current upper body posture. short, constants of Skeleton class. NO_SHAPE if posture was not updated */
	public short getCurrentUpperBodyPosture() {
		if (postureEvaluated)
			return currentUpperBodyPosture; 
		else
			return currentUpperBodyPosture=Skeleton.NO_POSE;
	}
	private boolean evaluateIShape() {
		float angleLArm = PVector.angleBetween(lUpperArmLocal,lLowerArmLocal);
		float angleRArm = PVector.angleBetween(rUpperArmLocal,rLowerArmLocal);
		float angleIShape = PVector.angleBetween(lUpperArmLocal, rUpperArmLocal);
		float angleTolerance = PConstants.PI*postureTolerance;
		if (isValueBetween(angleLArm,0,PApplet.radians(10)+angleTolerance) && isValueBetween(angleRArm,0,PApplet.radians(10)+angleTolerance)) { // arms form a straight line
			if (isValueBetween(angleIShape,0,PApplet.radians(15)+angleTolerance)) { // arms are parallel
				return true;
			}
		}
		return false;
	}
	private boolean evaluateOShape() {
		PVector rHandlHand = PVector.sub(jointsLocal[RIGHT_HAND],jointsLocal[LEFT_HAND]);
		if (isValueBetween(rHandlHand.mag(),0,100+(100*postureTolerance))) {
			float angleLUpper = PVector.angleBetween(lUpperArmLocal,orientationY);
			float angleRUpper = PVector.angleBetween(rUpperArmLocal,orientationY);
			float angleLLower = PVector.angleBetween(lLowerArmLocal,lUpperArmLocal);
			float angleRLower = PVector.angleBetween(rLowerArmLocal,rUpperArmLocal);
			float angleTolerance = PConstants.PI*postureTolerance;
			if (isValueBetween(angleLUpper,PApplet.radians(40)-angleTolerance,PApplet.radians(50)+angleTolerance) && isValueBetween(angleRUpper,PApplet.radians(40)-angleTolerance,PApplet.radians(50)+angleTolerance)) { // ~45 degree
				if (isValueBetween(angleLLower,PApplet.radians(95)-angleTolerance,PApplet.radians(105)+angleTolerance) && isValueBetween(angleRLower,PApplet.radians(95)-angleTolerance,PApplet.radians(105)+angleTolerance)) { // ~100 degree
					return true;
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
		float angleL = PVector.angleBetween(lUpperArmLocal,lLowerArmLocal);
		float angleR = PVector.angleBetween(rUpperArmLocal,rLowerArmLocal);
		float angleNShape = PVector.angleBetween(lUpperArmLocal, rUpperArmLocal);
		float angleToBody = PVector.angleBetween(lLowerArmLocal,orientationY);
		float angleTolerance = PConstants.PI*postureTolerance;
		if (isValueBetween(angleL,PApplet.radians(85)-angleTolerance,PApplet.radians(95)+angleTolerance) && isValueBetween(angleR,PApplet.radians(85)-angleTolerance,PApplet.radians(95)+angleTolerance)) { // arms angle ~90 degree
			if (isValueBetween(angleNShape,PApplet.radians(170)-angleTolerance,PApplet.radians(180))) { // upper arms form a straight line
				if (isValueBetween(angleToBody,PApplet.radians(90),PApplet.radians(180))) {// arms downwards 
					return true;
				}
			}
		}
		return false;
	}
	private boolean evaluateUShape() {
		float angleL = PVector.angleBetween(lUpperArmLocal,lLowerArmLocal);
		float angleR = PVector.angleBetween(rUpperArmLocal,rLowerArmLocal);
		float angleUShape = PVector.angleBetween(lUpperArmLocal, rUpperArmLocal);
		float angleToBody = PVector.angleBetween(lLowerArmLocal,orientationY);
		float angleTolerance = PConstants.PI*postureTolerance;
		if (isValueBetween(angleL,PApplet.radians(85)-angleTolerance,PApplet.radians(95)+angleTolerance) && isValueBetween(angleR,PApplet.radians(85)-angleTolerance,PApplet.radians(95)+angleTolerance)) { // arms angle ~90 degree
			if (isValueBetween(angleUShape,PApplet.radians(170)-angleTolerance,PApplet.radians(180))) { // upper arms form a straight line
				if (isValueBetween(angleToBody,0,PApplet.radians(90))) {// arms upwards 
					return true;
				}
			}
		}
		return false;
	}
	private boolean evaluateAShape() {
		float angleL = PVector.angleBetween(lUpperArmLocal,lLowerArmLocal);
		float angleR = PVector.angleBetween(rUpperArmLocal,rLowerArmLocal);
		float angleAShape = PVector.angleBetween(lUpperArmLocal, rUpperArmLocal);
		float angleToBody = PVector.angleBetween(lUpperArmLocal,orientationY);
		float angleTolerance = PConstants.PI*postureTolerance;
		if (isValueBetween(angleL,0,PApplet.radians(10)+angleTolerance) && isValueBetween(angleR,0,PApplet.radians(10)+angleTolerance)) { // arms form a straight line
			if (isValueBetween(angleAShape,PApplet.radians(85)-angleTolerance,PApplet.radians(95)+angleTolerance)) { // arms angle ~90 degree
				if (isValueBetween(angleToBody,PApplet.radians(90),PApplet.radians(180))) {// arms downwards 
					return true;
				}
			}
		}
		return false;
	}
	private boolean evaluateVShape() {
		float angleL = PVector.angleBetween(lUpperArmLocal,lLowerArmLocal);
		float angleR = PVector.angleBetween(rUpperArmLocal,rLowerArmLocal);
		float angleVShape = PVector.angleBetween(lUpperArmLocal, rUpperArmLocal);
		float angleToBody = PVector.angleBetween(lUpperArmLocal,orientationY);
		float angleTolerance = PConstants.PI*postureTolerance;
		if (isValueBetween(angleL,0,PApplet.radians(10)+angleTolerance) && isValueBetween(angleR,0,PApplet.radians(10)+angleTolerance)) { // arms form a straight line
			if (isValueBetween(angleVShape,PApplet.radians(85)-angleTolerance,PApplet.radians(95)+angleTolerance)) { // arms angle ~90 degree
				if (isValueBetween(angleToBody,0,PApplet.radians(90))) {// arms upwards 
					return true;
				}
			}
		}
		return false;
	}
	private boolean evaluateHandsForwardDownPose() {
		float angleL = PVector.angleBetween(lUpperArmLocal,lLowerArmLocal);
		float angleR = PVector.angleBetween(rUpperArmLocal,rLowerArmLocal);
		float angleUpperArms = PVector.angleBetween(lUpperArmLocal, rUpperArmLocal);
		float angleDownward = PVector.angleBetween(lUpperArmLocal,orientationY);
		float angleForward = PVector.angleBetween(lUpperArmLocal,orientationZ);
		float angleTolerance = PConstants.PI*postureTolerance;
		if (isValueBetween(angleL,0,PApplet.radians(10)+angleTolerance) && isValueBetween(angleR,0,PApplet.radians(10)+angleTolerance)) { // arms form a straight line
			if (isValueBetween(angleUpperArms,0,PApplet.radians(10)+angleTolerance)) { // arms are parallel
				if (isValueBetween(angleDownward,PApplet.radians(130)-angleTolerance,PApplet.radians(140)+angleTolerance)) {// arms downward 45 degree 
					if (isValueBetween(angleForward,0,PApplet.radians(90))) {// arms forward 
						return true;
					}
				}
			}
		}
		return false;
	}

	// -----------------------------------------------------------------
	// PRIVATE MATHS FUNCTIONS
	// evaluate local coord sys
	// origin: neck
	// orientation: 
	// +x==right_shoulder-left_shoulder, 
	// -y==orthogonal on +x and pointing to torso
	// +z==cross product of x and y
	private void calculateLocalCoordSys () {
		localCoordSysCalculated = false;
		origin = joints[Skeleton.NECK];
		
		// *** calculating local coordSys
		// +x-axis==right_shoulder-left_shoulder, 
		orientationX = PVector.sub(joints[Skeleton.RIGHT_SHOULDER],joints[Skeleton.LEFT_SHOULDER]);
		// +y==orthogonal to +x-axis, pointing from torso to x-axis. 
		// task: find point on orientationX
		// - the plane that contains torso and has orientationX as normal vector is defined as: 
		// - x1*orientationX.x+x2*orientationX.y+x3*orientationX.z=torso(dot)orientationX
		// - straight line defined by orientationX: [x] = left_shoulder+lambda*orientationX
		// - find lambda of crosspoint of that line with the plane: insert straight line as X into plane
		// - use lambda in straight line equation to get crosspoint
		// - +y is crosspoint-torso
		float lambda = joints[Skeleton.TORSO].dot(orientationX);
		lambda -= orientationX.dot(joints[Skeleton.LEFT_SHOULDER]); 
		lambda /= orientationX.dot(orientationX);
		PVector crossPoint = PVector.add(joints[Skeleton.LEFT_SHOULDER],PVector.mult(orientationX,lambda));
		orientationY = PVector.sub(crossPoint,joints[Skeleton.TORSO]);
		// =z-axis is cross product of y and x axis
		orientationZ = orientationY.cross(orientationX);
		
		orientationX.normalize();
		orientationY.normalize();
		orientationZ.normalize();
		
		transformCoordSys = new PMatrix3D (orientationX.x,orientationY.x,orientationZ.x,origin.x,
																 orientationX.y,orientationY.y,orientationZ.y,origin.y,
																 orientationX.z,orientationY.z,orientationZ.z,origin.z,
																 0f,0f,0f,1f);
		transformCoordSysInv = new PMatrix3D(transformCoordSys);
		transformCoordSysInv.invert();
		
		localCoordSysCalculated = true;
	}
	// transform joint coordinates to lokal coordsys. 
	private PVector getLocalVector (PVector globalVector) {
		if (localCoordSysCalculated) {
			PVector localVector = new PVector();
			transformCoordSysInv.mult(globalVector,localVector);
			return localVector;
		} else {
			PApplet.println("getLocalVector(): local coordsys not calculated yet!");
			return new PVector();
		}
	}
	// mirror joints
	private void mirrorOrientationMatrix (PMatrix3D matrix) {
		PVector x = new PVector(matrix.m00,matrix.m10,matrix.m20);
		PVector y = new PVector(matrix.m01,matrix.m11,matrix.m21);
		PVector z = new PVector(matrix.m02,matrix.m12,matrix.m22);
		x.add(rMP);
		y.add(rMP);
		z.add(rMP);
		float distanceToMP = PVector.dot(x,n0MP) - dMP;
		x.set(PVector.add(x,PVector.mult(n0MP,-2*distanceToMP)));
		distanceToMP = PVector.dot(y,n0MP) - dMP;
		y.set(PVector.add(y,PVector.mult(n0MP,-2*distanceToMP)));
		distanceToMP = PVector.dot(z,n0MP) - dMP;
		z.set(PVector.add(z,PVector.mult(n0MP,-2*distanceToMP)));
		x.sub(rMP);
		y.sub(rMP);
		z.sub(rMP);
		matrix.set(-x.x,y.x,z.x,matrix.m03,-x.y,y.y,z.y,matrix.m13,-x.z,y.z,z.z,matrix.m23,matrix.m30,matrix.m31,matrix.m32,matrix.m33);		
	}
	private void calculateMirrorPlane () {
		// calculate body plane of Shoulder and Torso points in HNF
		// HNF: r*n0-d = 0
		PVector r;
		PVector n0;
		// r is position vector of any point in the plane
		r = joints[Skeleton.TORSO];
		// n0 is cross product of two vectors in the plane
		PVector temp1 = PVector.sub(joints[Skeleton.LEFT_SHOULDER],r);
		PVector temp2 = PVector.sub(joints[Skeleton.RIGHT_SHOULDER],r);
		n0 = temp1.cross(temp2);
		n0.normalize();
		// mirrorPlane is orthogonal to body plane and contains the line between torso and neck
		// calculate mirrorPlane in HNF: r*n0-d = 0
		rMP = r; // r is always set to position of the torso
		n0MP = n0.cross(PVector.sub(joints[Skeleton.NECK],joints[Skeleton.TORSO]));
		n0MP.normalize();
		dMP = PVector.dot(rMP,n0MP);
		mirrorPlaneCalculated = true;
	}
	private boolean isValueBetween (float val, float lowerBound, float upperBound) {
			return (val >= lowerBound && val <= upperBound);
	}
	
	// -----------------------------------------------------------------
	// PRIVATE HELPER METHODS
	private void updateJointPositions () {
		confidenceJoints[Skeleton.HEAD] = kinect.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_HEAD,joints[Skeleton.HEAD]);
		confidenceJoints[Skeleton.NECK] = kinect.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_NECK,joints[Skeleton.NECK]);
		confidenceJoints[Skeleton.LEFT_SHOULDER] = kinect.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_LEFT_SHOULDER,joints[Skeleton.LEFT_SHOULDER]);
		confidenceJoints[Skeleton.RIGHT_SHOULDER] = kinect.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_RIGHT_SHOULDER,joints[Skeleton.RIGHT_SHOULDER]);
		confidenceJoints[Skeleton.TORSO] = kinect.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_TORSO,joints[Skeleton.TORSO]);
		confidenceJoints[Skeleton.LEFT_ELBOW] = kinect.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_LEFT_ELBOW,joints[Skeleton.LEFT_ELBOW]);
		confidenceJoints[Skeleton.LEFT_HAND] = kinect.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_LEFT_HAND,joints[Skeleton.LEFT_HAND]);
		confidenceJoints[Skeleton.RIGHT_ELBOW] = kinect.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_RIGHT_ELBOW,joints[Skeleton.RIGHT_ELBOW]);
		confidenceJoints[Skeleton.RIGHT_HAND] = kinect.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_RIGHT_HAND,joints[Skeleton.RIGHT_HAND]);
		if (fullBodyTracking) {
			confidenceJoints[Skeleton.LEFT_HIP] = kinect.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_LEFT_HIP,joints[Skeleton.LEFT_HIP]);
			confidenceJoints[Skeleton.LEFT_KNEE] = kinect.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_LEFT_KNEE,joints[Skeleton.LEFT_KNEE]);
			confidenceJoints[Skeleton.LEFT_FOOT] = kinect.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_LEFT_FOOT,joints[Skeleton.LEFT_FOOT]);
			confidenceJoints[Skeleton.RIGHT_HIP] = kinect.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_RIGHT_HIP,joints[Skeleton.RIGHT_HIP]);
			confidenceJoints[Skeleton.RIGHT_KNEE] = kinect.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_RIGHT_KNEE,joints[Skeleton.RIGHT_KNEE]);
			confidenceJoints[Skeleton.RIGHT_FOOT] = kinect.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_RIGHT_FOOT,joints[Skeleton.RIGHT_FOOT]);
		}
		lUpperArm = PVector.sub(joints[LEFT_ELBOW],joints[LEFT_SHOULDER]);
		rUpperArm = PVector.sub(joints[RIGHT_ELBOW],joints[RIGHT_SHOULDER]);
		lLowerArm = PVector.sub(joints[LEFT_HAND],joints[LEFT_ELBOW]);
		rLowerArm = PVector.sub(joints[RIGHT_HAND],joints[RIGHT_ELBOW]);
	}
	private void updateMirroredJointPositions () {
		if (mirrorPlaneCalculated) {
			switch (mirrorTherapy) {
				case MIRROR_THERAPY_LEFT:
					// mirror left elbow to right elbow
					float lDistanceToMP = PVector.dot(joints[Skeleton.LEFT_ELBOW],n0MP) - dMP;
					joints[Skeleton.RIGHT_ELBOW].set(PVector.add(joints[Skeleton.LEFT_ELBOW],PVector.mult(n0MP,-2*lDistanceToMP)));
					confidenceJoints[Skeleton.RIGHT_ELBOW] = confidenceJoints[Skeleton.LEFT_ELBOW];
					// mirror left hand to right hand
					lDistanceToMP = PVector.dot(joints[Skeleton.LEFT_HAND],n0MP) - dMP;
					joints[Skeleton.RIGHT_HAND].set(PVector.add(joints[Skeleton.LEFT_HAND],PVector.mult(n0MP,-2*lDistanceToMP)));
					confidenceJoints[Skeleton.RIGHT_HAND] = confidenceJoints[Skeleton.LEFT_HAND];
					break;
				case MIRROR_THERAPY_RIGHT:
					// mirror right elbow to left elbow
					float rDistanceToMP = PVector.dot(joints[Skeleton.RIGHT_ELBOW],n0MP) - dMP;
					joints[Skeleton.LEFT_ELBOW].set(PVector.add(joints[Skeleton.RIGHT_ELBOW],PVector.mult(n0MP,-2*rDistanceToMP)));
					confidenceJoints[Skeleton.LEFT_ELBOW] = confidenceJoints[Skeleton.RIGHT_ELBOW];
					// mirror right hand to left hand
					rDistanceToMP = PVector.dot(joints[Skeleton.RIGHT_HAND],n0MP) - dMP;
					joints[Skeleton.LEFT_HAND].set(PVector.add(joints[Skeleton.RIGHT_HAND],PVector.mult(n0MP,-2*rDistanceToMP)));
					confidenceJoints[Skeleton.LEFT_HAND] = confidenceJoints[Skeleton.RIGHT_HAND];
					break;
			}	
		}
	}
	private void updateJointOrientations () {
		confidenceJointOrientations[Skeleton.HEAD] = kinect.getJointOrientationSkeleton(userId,SimpleOpenNI.SKEL_HEAD,jointOrientations[Skeleton.HEAD]);
		confidenceJointOrientations[Skeleton.NECK] = kinect.getJointOrientationSkeleton(userId,SimpleOpenNI.SKEL_NECK,jointOrientations[Skeleton.NECK]);
		confidenceJointOrientations[Skeleton.LEFT_SHOULDER] = kinect.getJointOrientationSkeleton(userId,SimpleOpenNI.SKEL_LEFT_SHOULDER,jointOrientations[Skeleton.LEFT_SHOULDER]);
		confidenceJointOrientations[Skeleton.RIGHT_SHOULDER] = kinect.getJointOrientationSkeleton(userId,SimpleOpenNI.SKEL_RIGHT_SHOULDER,jointOrientations[Skeleton.RIGHT_SHOULDER]);
		confidenceJointOrientations[Skeleton.TORSO] = kinect.getJointOrientationSkeleton(userId,SimpleOpenNI.SKEL_TORSO,jointOrientations[Skeleton.TORSO]);
		confidenceJointOrientations[Skeleton.LEFT_ELBOW] = kinect.getJointOrientationSkeleton(userId,SimpleOpenNI.SKEL_LEFT_ELBOW,jointOrientations[Skeleton.LEFT_ELBOW]);
		confidenceJointOrientations[Skeleton.LEFT_HAND] = kinect.getJointOrientationSkeleton(userId,SimpleOpenNI.SKEL_LEFT_HAND,jointOrientations[Skeleton.LEFT_HAND]);
		confidenceJointOrientations[Skeleton.RIGHT_ELBOW] = kinect.getJointOrientationSkeleton(userId,SimpleOpenNI.SKEL_RIGHT_ELBOW,jointOrientations[Skeleton.RIGHT_ELBOW]);
		confidenceJointOrientations[Skeleton.RIGHT_HAND] = kinect.getJointOrientationSkeleton(userId,SimpleOpenNI.SKEL_RIGHT_HAND,jointOrientations[Skeleton.RIGHT_HAND]);
		if (fullBodyTracking) {
			confidenceJointOrientations[Skeleton.LEFT_HIP] = kinect.getJointOrientationSkeleton(userId,SimpleOpenNI.SKEL_LEFT_HIP,jointOrientations[Skeleton.LEFT_HIP]);
			confidenceJointOrientations[Skeleton.LEFT_KNEE] = kinect.getJointOrientationSkeleton(userId,SimpleOpenNI.SKEL_LEFT_KNEE,jointOrientations[Skeleton.LEFT_KNEE]);
			confidenceJointOrientations[Skeleton.LEFT_FOOT] = kinect.getJointOrientationSkeleton(userId,SimpleOpenNI.SKEL_LEFT_FOOT,jointOrientations[Skeleton.LEFT_FOOT]);
			confidenceJointOrientations[Skeleton.RIGHT_HIP] = kinect.getJointOrientationSkeleton(userId,SimpleOpenNI.SKEL_RIGHT_HIP,jointOrientations[Skeleton.RIGHT_HIP]);
			confidenceJointOrientations[Skeleton.RIGHT_KNEE] = kinect.getJointOrientationSkeleton(userId,SimpleOpenNI.SKEL_RIGHT_KNEE,jointOrientations[Skeleton.RIGHT_KNEE]);
			confidenceJointOrientations[Skeleton.RIGHT_FOOT] = kinect.getJointOrientationSkeleton(userId,SimpleOpenNI.SKEL_RIGHT_FOOT,jointOrientations[Skeleton.RIGHT_FOOT]);
		}
	}
	private void updateMirroredJointOrientations () {
		if (mirrorPlaneCalculated) {
			switch (mirrorTherapy) {
				case MIRROR_THERAPY_LEFT:
					// mirror orientation of left shoulder to right shoulder
					jointOrientations[Skeleton.RIGHT_SHOULDER].set(jointOrientations[Skeleton.LEFT_SHOULDER]);
					mirrorOrientationMatrix(jointOrientations[Skeleton.RIGHT_SHOULDER]);
					confidenceJointOrientations[Skeleton.RIGHT_SHOULDER] = confidenceJointOrientations[Skeleton.LEFT_SHOULDER];
					// mirror orientation of left elbow to right elbow
					jointOrientations[Skeleton.RIGHT_ELBOW].set(jointOrientations[Skeleton.LEFT_ELBOW]);
					mirrorOrientationMatrix(jointOrientations[Skeleton.RIGHT_ELBOW]);
					confidenceJointOrientations[Skeleton.RIGHT_ELBOW] = confidenceJointOrientations[Skeleton.LEFT_ELBOW];
					// mirror orientation of left hand to right hand
					jointOrientations[Skeleton.RIGHT_HAND] = jointOrientations[Skeleton.LEFT_HAND];
					mirrorOrientationMatrix(jointOrientations[Skeleton.RIGHT_HAND]);
					confidenceJointOrientations[Skeleton.RIGHT_HAND] = confidenceJointOrientations[Skeleton.LEFT_HAND];
					break;
				case MIRROR_THERAPY_RIGHT:
					// mirror orientation of right  shoulder to left shoulder
					jointOrientations[Skeleton.LEFT_SHOULDER].set(jointOrientations[Skeleton.RIGHT_SHOULDER]);
					mirrorOrientationMatrix(jointOrientations[Skeleton.LEFT_SHOULDER]);
					confidenceJointOrientations[Skeleton.LEFT_SHOULDER] = confidenceJointOrientations[Skeleton.RIGHT_SHOULDER];
					// mirror orientation of right  elbow to left elbow
					jointOrientations[Skeleton.LEFT_ELBOW] = jointOrientations[Skeleton.RIGHT_ELBOW]; 
					mirrorOrientationMatrix(jointOrientations[Skeleton.LEFT_ELBOW]);
					confidenceJointOrientations[Skeleton.LEFT_ELBOW] = confidenceJointOrientations[Skeleton.RIGHT_ELBOW];
					// mirror orientation of right  hand to left hand
					jointOrientations[Skeleton.LEFT_HAND] = jointOrientations[Skeleton.RIGHT_HAND];
					mirrorOrientationMatrix(jointOrientations[Skeleton.LEFT_HAND]);
					confidenceJointOrientations[Skeleton.LEFT_HAND] = confidenceJointOrientations[Skeleton.RIGHT_HAND];
					break;
			}
		}	
	}
	private void transformToLocalCoordSys () {
		jointsLocal[Skeleton.HEAD] = getLocalVector(joints[Skeleton.HEAD]);
		jointsLocal[Skeleton.NECK] = getLocalVector(joints[Skeleton.NECK]);
		jointsLocal[Skeleton.TORSO] = getLocalVector(joints[Skeleton.TORSO]);
		jointsLocal[Skeleton.LEFT_SHOULDER] = getLocalVector(joints[Skeleton.LEFT_SHOULDER]);
		jointsLocal[Skeleton.RIGHT_SHOULDER] = getLocalVector(joints[Skeleton.RIGHT_SHOULDER]);
		jointsLocal[Skeleton.LEFT_ELBOW] = getLocalVector(joints[Skeleton.LEFT_ELBOW]);
		jointsLocal[Skeleton.RIGHT_ELBOW] = getLocalVector(joints[Skeleton.RIGHT_ELBOW]);
		jointsLocal[Skeleton.LEFT_HAND] = getLocalVector(joints[Skeleton.LEFT_HAND]);
		jointsLocal[Skeleton.RIGHT_HAND] = getLocalVector(joints[Skeleton.RIGHT_HAND]);
		if (fullBodyTracking) {
			jointsLocal[Skeleton.LEFT_HIP] = getLocalVector(joints[Skeleton.LEFT_HIP]);
			jointsLocal[Skeleton.LEFT_KNEE] = getLocalVector(joints[Skeleton.LEFT_KNEE]);
			jointsLocal[Skeleton.LEFT_FOOT] = getLocalVector(joints[Skeleton.LEFT_FOOT]);
			jointsLocal[Skeleton.RIGHT_HIP] = getLocalVector(joints[Skeleton.RIGHT_HIP]);
			jointsLocal[Skeleton.RIGHT_KNEE] = getLocalVector(joints[Skeleton.RIGHT_KNEE]);
			jointsLocal[Skeleton.RIGHT_FOOT] = getLocalVector(joints[Skeleton.RIGHT_FOOT]);
		}
		lUpperArmLocal = PVector.sub(jointsLocal[LEFT_ELBOW],jointsLocal[LEFT_SHOULDER]);
		rUpperArmLocal = PVector.sub(jointsLocal[RIGHT_ELBOW],jointsLocal[RIGHT_SHOULDER]);
		lLowerArmLocal = PVector.sub(jointsLocal[LEFT_HAND],jointsLocal[LEFT_ELBOW]);
		rLowerArmLocal = PVector.sub(jointsLocal[RIGHT_HAND],jointsLocal[RIGHT_ELBOW]);
	}
}
	