/*
Copyright (c) 2012, Thomas Schueler, http://www.thomasschueler.de
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the author nor the
      names of the contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THOMAS SCHUELER BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

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
	
	// The interface to talk to kinect
	private SimpleOpenNI kinect;
	
	// stores skeleton Points in 3d Space, global coordsys
	private PVector[] joints = new PVector[15]; 
	private float[] confidenceJoints = new float[15];
	// stores skeleton Points in 3d Space, local coordsys (neck is origin)
	private PVector[] jointsLocal = new PVector[15]; 
	// stores joint orientation
	private PMatrix3D[] jointOrientations = new PMatrix3D[15];
	private float[] confidenceJointOrientations = new float[15];
	// for convenience store vectors of upper arms and lower arms
	private PVector lUpperArm = new PVector();
	private PVector lLowerArm = new PVector();
	private PVector rUpperArm = new PVector();
	private PVector rLowerArm = new PVector();
	private PVector lUpperArmLocal = new PVector();
	private PVector lLowerArmLocal = new PVector();
	private PVector rUpperArmLocal = new PVector();
	private PVector rLowerArmLocal = new PVector();
	
	// setup variables
	private boolean calculateLocalCoordSys = true;
	private boolean fullBodyTracking = true;
	private short mirrorTherapy = MIRROR_THERAPY_OFF;
	private boolean evaluatePostureAndGesture = true;
	// control state of skeleton
	private boolean isUpdated = false;
	private int updateCycle = 0; // count the number of update calls for time relevant calculations 
	// skeleton of user
	private int userId;

	// helper objects
	private SkeletonGesture gesture = null;
	private boolean gestureEvaluated = false;
	private SkeletonPosture posture = null;
	private boolean postureEvaluated = false;
	private SkeletonMath math = null;
	private boolean mirrorPlaneCalculated = false;
	private boolean localCoordSysCalculated = false;
	
	// -----------------------------------------------------------------
	// CONSTRUCTORS AND STATECONTROL
	/** Constructor for the Skeleton.
	 *  @param _kinect Handle to the SimpleOpenNI object. Skeleton will maintain its status in the update method by talking to SimpleOpenNI directly.
	 *  @param _userId the user ID of the skeleton
	 *  @param _fullBodyTracking switches full body tracking on/off. If switched off, only upper body joints will be evaluated
	 *  @param _calculateLocalCoordSys switches calculation of the local coordinate system on/off. If switched on, local coordination system will be calculated and joints will be transformed to it 
	 *  @param _evaluatePostureAndGesture switches calculation of the posture and gesture on/off. Switching it on requires local coord sys to be calculated 
	 *  @param _mirrorTherapy Sets the skeleton to mirror one body side to the other. When mirrorTherapy is set on, mirrorPlane will be calculated. Short value should correspond to skeleton constants. If out of range, mirror therapy will be switched off */
	public Skeleton (SimpleOpenNI _kinect, int _userId, boolean _fullBodyTracking, boolean _calculateLocalCoordSys, boolean _evaluatePostureAndGesture, short _mirrorTherapy) {
		kinect = _kinect;
		userId = _userId;
		fullBodyTracking = _fullBodyTracking;
		calculateLocalCoordSys = _calculateLocalCoordSys;
		evaluatePostureAndGesture = _evaluatePostureAndGesture;
		if (_mirrorTherapy >= MIRROR_THERAPY_OFF && _mirrorTherapy <= MIRROR_THERAPY_RIGHT) mirrorTherapy = _mirrorTherapy;
		setup();
	}
	/** Constructor for the Skeleton. Defaults used for setup.
	 *  @param _kinect Handle to the SimpleOpenNI object. Skeleton will maintain its status in the update method by talking to SimpleOpenNI directly.
	 *  @param _userId the user ID of the skeleton */
	public Skeleton (SimpleOpenNI _kinect, int _userId) {
		kinect = _kinect;
		userId = _userId;
		setup();
	}
	// helper for constructor
	private void setup () {
		for (int i=0; i<15; i++){
			joints[i] = new PVector();
			jointsLocal[i] = new PVector();
			jointOrientations[i] = new PMatrix3D();
		}
		posture = new SkeletonPosture(this);
		gesture = new SkeletonGesture(this);
		math = new SkeletonMath(this);
	}

	/** Update method. Call it to update status of skeleton. Skeleton will talk to SimpleOpenNI directly and will do all the necessary math for updating its status according to set up */
	public void update () {
		isUpdated = false;
		localCoordSysCalculated = false;
		mirrorPlaneCalculated = false;
		gestureEvaluated = false;
		postureEvaluated = false;
		
		updateJointPositions();
		updateJointOrientations();
		
		if (mirrorTherapy != MIRROR_THERAPY_OFF) {
			math.calculateMirrorPlane();
			mirrorPlaneCalculated = true;
			updateMirroredJointPositions();
			updateMirroredJointOrientations();
		}
		if (calculateLocalCoordSys) {
			math.calculateLocalCoordSys();
			localCoordSysCalculated = true;
			transformToLocalCoordSys();
			if (evaluatePostureAndGesture) {
				posture.evaluate();
				postureEvaluated = true;
				gesture.evaluate(updateCycle);
				gestureEvaluated = true;
			}
		}
		
		updateCycle++;
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
	/** Setter for the tolerance with which gesture will be detected. 0..1f.
	 *  @param the tolerance between 0..1f. when higher than 1 or lower than 0, default tolerance 0.5f will be set */
	public void setGestureTolerance (float _gestureTolerance) {
		if (gesture != null) {
			gesture.setGestureTolerance(_gestureTolerance);
		}
	}
	/** Getter for gesture tolerance
	 *  @return the gesture tolerance or -1f if gesture evaluation is not activated*/
	public float getGestureTolerance () {
		if (gesture != null) {
			return gesture.getGestureTolerance();
		} else {
			return -1f;
		}
	}
	/** Setter for the tolerance with which posture will be detected. 0..1f.
	 *  @param the tolerance between 0..1f. when higher than 1 or lower than 0, default tolerance 0.3f will be set */
	public void setPostureTolerance (float _postureTolerance) {
		if (posture != null){
			posture.setPostureTolerance(_postureTolerance);
		}
	}
	/** Getter for posture tolerance
	 *  @return the posture tolerance or -1f if posture evaluation is not activated */
	public float getPostureTolerance () {
		if (posture != null) {
			return posture.getPostureTolerance();
		} else {
			return -1f;
		}
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
	public PVector getJointLCS (short jointType) {
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
	/** The angle between the left upper Arm and the body axis. 
	 *  @return The angle between the left upper Arm and the body axis.*/
	public float getAngleLeftUpperArm() {
		return PVector.angleBetween(lUpperArmLocal,getOrientationY());
	}
	/** The angle between the left lower Arm and the left upper arm. 
	 *  @return The angle between the left lower Arm and the left upper arm.*/
	public float getAngleLeftLowerArm() {
		return PVector.angleBetween(lLowerArmLocal,lUpperArmLocal);
	}
	/** The angle between the right upper Arm and the body axis. 
	 *  @return The angle between the right upper Arm and the body axis.*/
	public float getAngleRightUpperArm() {
		return PVector.angleBetween(rUpperArmLocal,getOrientationY());
	}
	/** The angle between the right lower Arm and the right upper arm. 
	 *  @return The angle between the right lower Arm and the right upper arm.*/
	public float getAngleRightLowerArm() {
		return PVector.angleBetween(rLowerArmLocal,rUpperArmLocal);
	}
	/** The vectors for the lower and upper arms are calculated for convenience. The arms in the local coordinate system are calculated only if calculateLocalCoordSys was set.
	 *  @return The vector for the left upper arm in the local coordinate system. If localCoordSys was not calculated: 0-vector */
	public PVector getLeftUpperArmLCS() {
		if (localCoordSysCalculated)
			return lUpperArmLocal;
		else
			return new PVector();
	}
	/** The vectors for the lower and upper arms are calculated for convenience. The arms in the local coordinate system are calculated only if calculateLocalCoordSys was set.
	 *  @return The vector for the left lower arm in the local coordinate system. If localCoordSys was not calculated: 0-vector */
	public PVector getRightUpperArmLCS() {
		if (localCoordSysCalculated)
			return lLowerArmLocal;
		else
			return new PVector();
	}
	/** The vectors for the lower and upper arms are calculated for convenience. The arms in the local coordinate system are calculated only if calculateLocalCoordSys was set.
	 *  @return The vector for the right upper arm in the local coordinate system. If localCoordSys was not calculated: 0-vector */
	public PVector getLeftLowerArmLCS() {
		if (localCoordSysCalculated)
			return rUpperArmLocal;
		else
			return new PVector();
	}
	/** The vectors for the lower and upper arms are calculated for convenience. The arms in the local coordinate system are calculated only if calculateLocalCoordSys was set.
	 *  @return The vector for the right lower arm in the local coordinate system. If localCoordSys was not calculated: 0-vector */
	public PVector getRightLowerArmLCS() {
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
			return math.getRMP();
		else
			return new PVector();
	}
	/** Mirror Plane is defined in HNF: r*n0-d = 0. works only if mirror plane has been calculated, i.e. skeleton is in mirror therapy mode.
	 *  @return the normal vector of the mirrorPlane if mirror plane was calculated. Else 0-vector */
	public PVector getN0VectorMirrorPlane () {
		if (mirrorPlaneCalculated)
			return math.getN0MP();
		else
			return new PVector();
	}
	/** Mirror Plane is defined in HNF: r*n0-d = 0. works only if mirror plane has been calculated, i.e. skeleton is in mirror therapy mode.
	 *  @return the distance of the mirrorPlane to the origin if mirror plane was calculated. Else 0f */
	public float getDValueMirrorPlane () {
		if (mirrorPlaneCalculated)
			return math.getDMP();
		else
			return 0f;
	}
	/** returns the origin of the local coordsys. Equals Torso Vector. works only if localCoordSys has been calculated
	 *  @return the origin. 0-vector if localCoordSys has not been calculated */
	public PVector getOrigin () {
		if (localCoordSysCalculated)
			return math.getOrigin();
		else
			return new PVector();
	}
	/** returns the local x coordinate vector. works only if localCoordSys has been calculated
	 *  @return the local vector. 0-vector if localCoordSys has not been calculated */
	public PVector getOrientationX () {
		if (localCoordSysCalculated)
			return math.getOrientationX();
		else
			return new PVector();
	}
	/** returns the local x coordinate vector projected to the kinects projectionn plane. works only if localCoordSys has been calculated
	 *  @return the local vector projected to the kinects projection plane. 0-vector if localCoordSys has not been calculated */
	public PVector getOrientationXProjective () {
		PVector projective = new PVector();
		kinect.convertRealWorldToProjective(getOrientationX(),projective);
		return projective;
	}
	/** returns the angle between the local x vector and the global x vector. works only if localCoordSys has been calculated
	 *  @return the angle, float between 0 and PI. 0f when localCoordSys has not been calculated */
	public float getOrientationAlpha () {
		return PVector.angleBetween(getOrientationX(),new PVector(1,0,0));
	}
	/** returns the local y coordinate vector. works only if localCoordSys has been calculated
	 *  @return the local vector. 0-vector if localCoordSys has not been calculated */
	public PVector getOrientationY () {
		if (localCoordSysCalculated)
			return math.getOrientationY();
		else
			return new PVector();
	}
	/** returns the local y coordinate vector projected to the kinects projectionn plane. works only if localCoordSys has been calculated
	 *  @return the local vector projected to the kinects projection plane. 0-vector if localCoordSys has not been calculated */
	public PVector getOrientationYProjective () {
		PVector projective = new PVector();
		kinect.convertRealWorldToProjective(getOrientationY(),projective);
		return projective;
	}
	/** returns the angle between the local y vector and the global y vector. works only if localCoordSys has been calculated
	 *  @return the angle, float between 0 and PI. 0f when localCoordSys has not been calculated */
	public float getOrientationBeta () {
		return PVector.angleBetween(getOrientationY(),new PVector(0,1,0));
	}
	/** returns the local z coordinate vector. works only if localCoordSys has been calculated
	 *  @return the local vector. 0-vector if localCoordSys has not been calculated */
	public PVector getOrientationZ () {
		if (localCoordSysCalculated)
			return math.getOrientationZ();
		else
			return new PVector();
	}
	/** returns the local z coordinate vector projected to the kinects projectionn plane. works only if localCoordSys has been calculated
	 *  @return the local vector projected to the kinects projection plane. 0-vector if localCoordSys has not been calculated */
	public PVector getOrientationZProjective () {
		PVector projective = new PVector();
		kinect.convertRealWorldToProjective(getOrientationZ(),projective);
		return projective;
	}
	/** returns the angle between the local z vector and the global z vector. works only if localCoordSys has been calculated
	 *  @return the angle, float between 0 and PI. 0f when localCoordSys has not been calculated*/
	public float getOrientationGamma () {
		return PVector.angleBetween(getOrientationZ(),new PVector(0,0,1));
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
	/** returns the angle between the limb-vector and local X axis (shoulder_l-shoulder_r)
	 *  @param joint11 the joint the limb-vector points to
	 *  @param joint12 the joint the limb-vector origins in
	 *  @return the angle, float between 0 and PI */
	public float angleToLocalXAxis (short joint11, short joint12) {
		PVector axis1 = PVector.sub(jointsLocal[joint11],jointsLocal[joint12]);
		return PVector.angleBetween(axis1,getOrientationX());
	}
	/** returns the angle between the limb-vector and local Y axis (neck-torso)
	 *  @param joint11 the joint the limb-vector points to
	 *  @param joint12 the joint the limb-vector origins in
	 *  @return the angle, float between 0 and PI */
	public float angleToLocalYAxis (short joint11, short joint12) {
		PVector axis1 = PVector.sub(jointsLocal[joint11],jointsLocal[joint12]);
		return PVector.angleBetween(axis1,getOrientationY());
	}
	/** returns the angle between the limb-vector and local Z axis (orthogonal on local x/y-plane)
	 *  @param joint11 the joint the limb-vector points to
	 *  @param joint12 the joint the limb-vector origins in
	 *  @return the angle, float between 0 and PI */
	public float angleToLocalZAxis (short joint11, short joint12) {
		PVector axis1 = PVector.sub(jointsLocal[joint11],jointsLocal[joint12]);
		return PVector.angleBetween(axis1,getOrientationZ());
	}
	/** returns the angle between the limb-vector and the global X axis
	 *  @param joint11 the joint the limb-vector points to
	 *  @param joint12 the joint the limb-vector origins in
	 *  @return the angle, float between 0 and PI */
	public float angleToGlobalXAxis (short joint11, short joint12) {
		PVector axis1 = PVector.sub(joints[joint11],joints[joint12]);
		return PVector.angleBetween(axis1,new PVector(1,0,0));
	}
	/** returns the angle between the limb-vector and the global Y axis
	 *  @param joint11 the joint the limb-vector points to
	 *  @param joint12 the joint the limb-vector origins in
	 *  @return the angle, float between 0 and PI */
	public float angleToGlobalYAxis (short joint11, short joint12) {
		PVector axis1 = PVector.sub(joints[joint11],joints[joint12]);
		return PVector.angleBetween(axis1,new PVector(0,1,0));
	}
	/** returns the angle between the limb-vector and the global Z axis
	 *  @param joint11 the joint the limb-vector points to
	 *  @param joint12 the joint the limb-vector origins in
	 *  @return the angle, float between 0 and PI */
	public float angleToGlobalZAxis (short joint11, short joint12) {
		PVector axis1 = PVector.sub(joints[joint11],joints[joint12]);
		return PVector.angleBetween(axis1,new PVector(0,0,1));
	}
	/** returns the distance of the skeletons torso joint to the kinect
	 *  @return the distance in mm, magnitude of skeletons torso vector */
	public float distanceToKinect () {
		return joints[Skeleton.TORSO].mag();
	}
	
	// -----------------------------------------------------------------
	// POSTURE AND GESTURE ACCESS
	/** Returns the posture of the skeleton, if calculation is activated and posture was evaluated in last update cycle. See class SkeletonPosture for details
	 *  @return current upper body posture. short, constants of SkeletonPosture class, NO_POSE if posture is not calculated */
	public short getCurrentUpperBodyPosture() {
		if (posture != null && postureEvaluated)
			return posture.getCurrentUpperBodyPosture(); 
		else
			return SkeletonPosture.NO_POSE;
	}
	/** Returns the last gesture evaluated within _lookAtPastUpdatedCycles, if calculation is activated and gesture was evaluated in last update cycle. See SkeletonGestures class for details
	 *  @param _lookAtPastUpdateCycles the number of past update cycles during which the gesture should have been recognized.
	 *  @return current upper body gesture. short, constants of SkeletonGestures class, NO_GESTURE if no gesture was recognized in the given past update cycles or gesture evaluation is switched off */
	public short getLastUpperBodyGesture (int _lookAtPastUpdateCycles) {
		if (gesture != null && gestureEvaluated)
			return gesture.getLastUpperBodyGesture(updateCycle-_lookAtPastUpdateCycles);
		else
			return SkeletonGesture.NO_GESTURE;
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
	
	private void updateMirroredJointPositions () {
		if (mirrorPlaneCalculated) {
			switch (mirrorTherapy) {
				case MIRROR_THERAPY_LEFT:
					// mirror left elbow to right elbow
					joints[Skeleton.RIGHT_ELBOW].set(math.mirrorJointVector(joints[Skeleton.LEFT_ELBOW]));
					confidenceJoints[Skeleton.RIGHT_ELBOW] = confidenceJoints[Skeleton.LEFT_ELBOW];
					// mirror left hand to right hand
					joints[Skeleton.RIGHT_HAND].set(math.mirrorJointVector(joints[Skeleton.LEFT_HAND]));
					confidenceJoints[Skeleton.RIGHT_HAND] = confidenceJoints[Skeleton.LEFT_HAND];
					break;
				case MIRROR_THERAPY_RIGHT:
					// mirror right elbow to left elbow
					joints[Skeleton.LEFT_ELBOW].set(math.mirrorJointVector(joints[Skeleton.RIGHT_ELBOW]));
					confidenceJoints[Skeleton.LEFT_ELBOW] = confidenceJoints[Skeleton.RIGHT_ELBOW];
					// mirror right hand to left hand
					joints[Skeleton.LEFT_HAND].set(math.mirrorJointVector(joints[Skeleton.RIGHT_HAND]));
					confidenceJoints[Skeleton.LEFT_HAND] = confidenceJoints[Skeleton.RIGHT_HAND];
					break;
			}	
		}
	}
	private void updateMirroredJointOrientations () {
		if (mirrorPlaneCalculated) {
			switch (mirrorTherapy) {
				case MIRROR_THERAPY_LEFT:
					// mirror orientation of left shoulder to right shoulder
					jointOrientations[Skeleton.RIGHT_SHOULDER].set(math.mirrorOrientationMatrix(jointOrientations[Skeleton.LEFT_SHOULDER]));
					confidenceJointOrientations[Skeleton.RIGHT_SHOULDER] = confidenceJointOrientations[Skeleton.LEFT_SHOULDER];
					// mirror orientation of left elbow to right elbow
					jointOrientations[Skeleton.RIGHT_ELBOW].set(math.mirrorOrientationMatrix(jointOrientations[Skeleton.LEFT_ELBOW]));
					confidenceJointOrientations[Skeleton.RIGHT_ELBOW] = confidenceJointOrientations[Skeleton.LEFT_ELBOW];
					// mirror orientation of left hand to right hand
					jointOrientations[Skeleton.RIGHT_HAND].set(math.mirrorOrientationMatrix(jointOrientations[Skeleton.LEFT_HAND]));
					confidenceJointOrientations[Skeleton.RIGHT_HAND] = confidenceJointOrientations[Skeleton.LEFT_HAND];
					break;
				case MIRROR_THERAPY_RIGHT:
					// mirror orientation of right  shoulder to left shoulder
					jointOrientations[Skeleton.LEFT_SHOULDER].set(math.mirrorOrientationMatrix(jointOrientations[Skeleton.RIGHT_SHOULDER]));
					confidenceJointOrientations[Skeleton.LEFT_SHOULDER] = confidenceJointOrientations[Skeleton.RIGHT_SHOULDER];
					// mirror orientation of right  elbow to left elbow
					jointOrientations[Skeleton.LEFT_ELBOW].set(math.mirrorOrientationMatrix(jointOrientations[Skeleton.RIGHT_ELBOW]));
					confidenceJointOrientations[Skeleton.LEFT_ELBOW] = confidenceJointOrientations[Skeleton.RIGHT_ELBOW];
					// mirror orientation of right  hand to left hand
					jointOrientations[Skeleton.LEFT_HAND].set(math.mirrorOrientationMatrix(jointOrientations[Skeleton.RIGHT_HAND]));
					confidenceJointOrientations[Skeleton.LEFT_HAND] = confidenceJointOrientations[Skeleton.RIGHT_HAND];
					break;
			}
		}	
	}
	private void transformToLocalCoordSys () {
		jointsLocal[Skeleton.HEAD] = math.getLocalVector(joints[Skeleton.HEAD]);
		jointsLocal[Skeleton.NECK] = math.getLocalVector(joints[Skeleton.NECK]);
		jointsLocal[Skeleton.TORSO] = math.getLocalVector(joints[Skeleton.TORSO]);
		jointsLocal[Skeleton.LEFT_SHOULDER] = math.getLocalVector(joints[Skeleton.LEFT_SHOULDER]);
		jointsLocal[Skeleton.RIGHT_SHOULDER] = math.getLocalVector(joints[Skeleton.RIGHT_SHOULDER]);
		jointsLocal[Skeleton.LEFT_ELBOW] = math.getLocalVector(joints[Skeleton.LEFT_ELBOW]);
		jointsLocal[Skeleton.RIGHT_ELBOW] = math.getLocalVector(joints[Skeleton.RIGHT_ELBOW]);
		jointsLocal[Skeleton.LEFT_HAND] = math.getLocalVector(joints[Skeleton.LEFT_HAND]);
		jointsLocal[Skeleton.RIGHT_HAND] = math.getLocalVector(joints[Skeleton.RIGHT_HAND]);
		if (fullBodyTracking) {
			jointsLocal[Skeleton.LEFT_HIP] = math.getLocalVector(joints[Skeleton.LEFT_HIP]);
			jointsLocal[Skeleton.LEFT_KNEE] = math.getLocalVector(joints[Skeleton.LEFT_KNEE]);
			jointsLocal[Skeleton.LEFT_FOOT] = math.getLocalVector(joints[Skeleton.LEFT_FOOT]);
			jointsLocal[Skeleton.RIGHT_HIP] = math.getLocalVector(joints[Skeleton.RIGHT_HIP]);
			jointsLocal[Skeleton.RIGHT_KNEE] = math.getLocalVector(joints[Skeleton.RIGHT_KNEE]);
			jointsLocal[Skeleton.RIGHT_FOOT] = math.getLocalVector(joints[Skeleton.RIGHT_FOOT]);
		}
		lUpperArmLocal = PVector.sub(jointsLocal[LEFT_ELBOW],jointsLocal[LEFT_SHOULDER]);
		rUpperArmLocal = PVector.sub(jointsLocal[RIGHT_ELBOW],jointsLocal[RIGHT_SHOULDER]);
		lLowerArmLocal = PVector.sub(jointsLocal[LEFT_HAND],jointsLocal[LEFT_ELBOW]);
		rLowerArmLocal = PVector.sub(jointsLocal[RIGHT_HAND],jointsLocal[RIGHT_ELBOW]);
	}
}
	