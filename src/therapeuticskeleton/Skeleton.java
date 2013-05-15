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

import java.io.BufferedWriter;

import SimpleOpenNI.SimpleOpenNI;
import processing.core.*;

/** Skeleton represents a humanoid skeleton in 3d space that is tracked by Microsoft Kinect/Primensense sensor, openNI driver and SimpleOpenNI Java Library. 
 *  As a primary difference to the build-in skeleton data of OpenNI, this Skeleton takes the anatomical viewpoint for left/right labeling of body parts. <br>
 *  Skeleton provides basic calculations on joint position and angles. It provides access to joint and limb vectors. 
 *  Skeleton is meant as Processing middleware library for therapeutic purposes. For neurologic therapy, mirroring of body sides is available.
 *  Detection of certain body posture and gestures is implemented. See classes SkeletonPosture and SkeletonGesture for details.
 *  Classes in this package (therapeuticskeleton) are not meant to be used alone. 
 *  Please contact me, if you want to know more or are willing to work on this project. (thschuel@uos.de)
 * */
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
	
	// clinical directions
	public static final short NEUTRAL = 0;
	public static final short SUPERIOR = 1;
	public static final short INFERIOR = 2;
	public static final short POSTERIOR = 3;
	public static final short ANTERIOR = 4;
	public static final short LEFT_LATERAL = 5;
	public static final short RIGHT_LATERAL = 6;
	
	// The interface to talk to kinect
	private SimpleOpenNI kinect;
	
	// stores skeleton Points in 3d Space, global coordsys
	private PVector[] joint = new PVector[15]; 
	private PVector[] jointUnmirrored = new PVector[15];
	private float[] jointConfidence = new float[15];
	private float[] jointConfidenceUnmirrored = new float[15];
	// stores skeleton Points in 3d Space, local coordsys (neck is origin)
	private PVector[] jointLCS = new PVector[15];
	private PVector[] jointLCSUnmirrored = new PVector[15];
	// stores joint orientation
	private PMatrix3D[] jointOrientation = new PMatrix3D[15];
	private PMatrix3D[] jointOrientationUnmirrored = new PMatrix3D[15];
	private float[] jointOrientationConfidence = new float[15];
	private float[] jointOrientationConfidenceUnmirrored = new float[15];
	// stores distance of joints to last position of joints
	private float[] jointDelta = new float[15];
	private float[] jointDeltaUnmirrored = new float[15];
	
	// for convenience store vectors of upper arms and lower arms
	private PVector lUpperArm = new PVector();
	private PVector lLowerArm = new PVector();
	private PVector rUpperArm = new PVector();
	private PVector rLowerArm = new PVector();
	private PVector lUpperArmLCS = new PVector();
	private PVector lLowerArmLCS = new PVector();
	private PVector rUpperArmLCS = new PVector();
	private PVector rLowerArmLCS = new PVector();
	private PVector lUpperArmUnmirrored = new PVector();
	private PVector lLowerArmUnmirrored = new PVector();
	private PVector rUpperArmUnmirrored = new PVector();
	private PVector rLowerArmUnmirrored = new PVector();
	private PVector lUpperArmLCSUnmirrored = new PVector();
	private PVector lLowerArmLCSUnmirrored = new PVector();
	private PVector rUpperArmLCSUnmirrored = new PVector();
	private PVector rLowerArmLCSUnmirrored = new PVector();
	
	// setup variables
	private boolean fullBodyTracking = true;
	private short mirrorTherapy = MIRROR_THERAPY_OFF;
	private boolean evaluatePostureAndGesture = true;
	private boolean evaluateStatistics = true;
	// control state of skeleton
	private boolean isUpdated = false;
	private int currentFrameCount = 0;
	private float currentFrameRate = 0;
	// skeleton of user
	private int userId;

	// helper objects
	private SkeletonGesture gesture = null;
	private boolean gestureEvaluated = false;
	private SkeletonPosture posture = null;
	private boolean postureEvaluated = false;
	private SkeletonMath math = null;
	private SkeletonStatistics statistics = null;
	
	// -----------------------------------------------------------------
	// CONSTRUCTORS AND STATECONTROL
	/** Constructor for the Skeleton.
	 *  @param _kinect Handle to the SimpleOpenNI object. Skeleton will maintain its status in the update method by talking to SimpleOpenNI directly.
	 *  @param _userId the user ID of the skeleton
	 *  @param _fullBodyTracking switches full body tracking on/off. If switched off, only upper body joints will be evaluated */
	public Skeleton (SimpleOpenNI _kinect, int _userId, boolean _fullBodyTracking) {
		kinect = _kinect;
		userId = _userId;
		fullBodyTracking = _fullBodyTracking;
		for (int i=0; i<15; i++){
			joint[i] = new PVector();
			jointLCS[i] = new PVector();
			jointOrientation[i] = new PMatrix3D();
			jointDelta[i] = 0f;
			jointUnmirrored[i] = new PVector();
			jointLCSUnmirrored[i] = new PVector();
			jointOrientationUnmirrored[i] = new PMatrix3D();
			jointDeltaUnmirrored[i] = 0f;
		}
		math = new SkeletonMath(this);
	}

	/** Update method. Call it to update status of skeleton. Skeleton will talk to SimpleOpenNI directly and will do all the necessary math for updating its status according to set up 
	 *  @param _frameCount the current frame of PApplet, used for statistics
	 *  @param _frameRate the current frame rate of PApplet, used for statistics */
	public void update (int _frameCount, float _frameRate) {
		isUpdated = false;
		gestureEvaluated = false;
		postureEvaluated = false;
		currentFrameCount = _frameCount;
		currentFrameRate = _frameRate;
		
		// Update all internal information
		updateJointPositions();
		updateJointOrientations();

		// the order is important since body planes use information of local coordinate system
		math.calculateLocalCoordSys();
		math.calculateBodyPlanes();
		if (mirrorTherapy != MIRROR_THERAPY_OFF) {
			updateMirroredJointPositions();
			updateMirroredJointOrientations();
		}
		
		// transform updated skeleton to local coordinate system
		transformToLCS();
		
		if (evaluatePostureAndGesture && posture != null && gesture != null) {
			posture.evaluate();
			postureEvaluated = true;
			gesture.evaluate(currentFrameCount);
			gestureEvaluated = true;
		}
		if (evaluateStatistics && statistics != null) {
			statistics.update(currentFrameCount,currentFrameRate);
		}
		isUpdated = true;
	}
	
	
	// -----------------------------------------------------------------
	// GETTERS AND SETTERS FOR SETUP VARIABLES
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
	/** Setter for evaluating statistics for the skeleton. If switched on, statistics will be updated
	 *  If was switched on before, new statistics object will be generated! Old statistics will be lost.
	 *  @param _evaluateStatistics switch to set evaluating statistics on/off 
	 *  @param _buffer the file buffer to which statistics will be written. if null, statistics will not be logged */
	public void setEvaluateStatistics (boolean _evaluateStatistics, BufferedWriter _buffer) {
		evaluateStatistics = _evaluateStatistics;
		if (evaluateStatistics) {
			statistics = new SkeletonStatistics(this);
			if (_buffer != null) {
				statistics.startStatisticsLogging(_buffer);
			}
		} else {
			if (statistics != null) {
				statistics.stopStatisticsLogging();
				statistics = null;
			}
		}
	}
	/** Getter for evaluating statistics for the skeleton.
	 *  @return boolean of evaluateStatistics-switch */
	public boolean getEvaluateStatistics () {
		return evaluateStatistics;
	}
	/** Setter for evaluating posture and gesture for the skeleton. If switched on, posture and gesture will be updated
	 *  @param _evaluatePostureAndGesture switch to set evaluating posture and gesture on/off */
	public void setEvaluatePostureAndGesture (boolean _evaluatePostureAndGesture) {
		evaluatePostureAndGesture = _evaluatePostureAndGesture;
		if (evaluatePostureAndGesture) {
			posture = new SkeletonPosture(this);
			gesture = new SkeletonGesture(this);
		} else {
			posture = null;
			gesture = null;
		}
	}
	/** Getter for evaluating posture and gesture for the skeleton.
	 *  @return boolean of evaluatePostureAndGesture-switch */
	public boolean getEvaluatePostureAndGesture () {
		return evaluatePostureAndGesture;
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
	 *  @param _gestureTolerance the tolerance between 0..1f. when higher than 1 or lower than 0, default tolerance 0.5f will be set */
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
	 *  @param _postureTolerance the tolerance between 0..1f. when higher than 1 or lower than 0, default tolerance 0.3f will be set */
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

	// -----------------------------------------------------------------
	// GETTERS FOR STATISTICS OF SKELETON
	/** Getter for SkeletonStatistics. The whole class object is returned to be displayed in the main applet.
	 *  This getter provides access to the updated statistics object.
	 *  @return the skeleton statistics object, null if statistics evaluation is switched off */
	public SkeletonStatistics getLiveStatistics () {
		return statistics;
	}
	/** Getter for SkeletonStatistics. The whole class object is returned to be displayed in the main applet.
	 *  This getter returns a copy of the statistics object that is not linked to a skeleton object and won't be further updated.
	 *  @return a copy of the skeleton statistics object, null if statistics evluation is switched off */
	public SkeletonStatistics getFinalStatistics () {
		if (statistics != null) {
			return new SkeletonStatistics(statistics);
		} else {
			return null;
		}
	}
	
	// -----------------------------------------------------------------
	// ACCESS TO JOINTS AND JOINT INFORMATION
	/** This method returns the joint position of a certain joint in the global coordinate system
	 *  @param jointType The joint for which confidence value should be returned. Should be a short value corresponding to Skeleton constants.
	 *  @return The position of a certain joint in the global coordinate system as vector. If jointType out of range: 0-vector */
	public PVector getJoint (short jointType) {
		PVector returnValue = new PVector();
		if (jointType >= 0 && jointType <= 14) returnValue.set(joint[jointType]);
		return returnValue;
	}
	/** This method returns the joint position of a certain joint in the global coordinate system. This method returns the unmirrored information, regardless of mirror therapy mode.
	 *  @param jointType The joint for which confidence value should be returned. Should be a short value corresponding to Skeleton constants.
	 *  @return The position of a certain joint in the global coordinate system as vector. If jointType out of range: 0-vector */
	public PVector getJointUnmirrored (short jointType) {
		PVector returnValue = new PVector();
		if (jointType >= 0 && jointType <= 14) returnValue.set(jointUnmirrored[jointType]);
		return returnValue;
	}
	/** This method returns the joint position of a certain joint on the kinect's projective plane. Z-value will be 0
	 *  @param jointType The joint for which confidence value should be returned. Should be a short value corresponding to Skeleton constants.
	 *  @return The position of a certain joint as vector on the projective plane of the kinect. If jointType out of range: 0-vector */
	public PVector getJointProjective (short jointType) {
		PVector projective = new PVector();
		if (jointType >= 0 && jointType <= 14) kinect.convertRealWorldToProjective(joint[jointType], projective);
		return projective;
	}
	/** The positions of the joints are transformed to the local coordinate system of the skeleton if calculateLocalCoordSys was set.
	 *  This method returns the joint position of a certain joint in the local coordinate system. Works only if localCoordSysCalculated is true.
	 *  @param jointType The joint for which confidence value should be returned. Should be a short value corresponding to Skeleton constants.
	 *  @return The position of a certain joint in the local coordinate system as vector. If jointType out of range or if localCoordSys was not calculated: 0-vector */
	public PVector getJointLCS (short jointType) {
		PVector returnValue = new PVector();
		if (jointType >= 0 && jointType <= 14) returnValue.set(jointLCS[jointType]);
		return returnValue;
	}
	/** The positions of the joints are transformed to the local coordinate system of the skeleton if calculateLocalCoordSys was set. This method returns the unmirrored information, regardless of mirror therapy mode.
	 *  This method returns the joint position of a certain joint in the local coordinate system. Works only if localCoordSysCalculated is true.
	 *  @param jointType The joint for which confidence value should be returned. Should be a short value corresponding to Skeleton constants.
	 *  @return The position of a certain joint in the local coordinate system as vector. If jointType out of range or if localCoordSys was not calculated: 0-vector */
	public PVector getJointLCSUnmirrored (short jointType) {
		PVector returnValue = new PVector();
		if (jointType >= 0 && jointType <= 14) returnValue.set(jointLCSUnmirrored[jointType]);
		return returnValue;
	}
	/** The positions of the joints are evaluated with a certain confidence value. This method returns the confidence value for a certain joint
	 *  @param jointType The joint for which confidence value should be returned. Should be a short value corresponding to Skeleton constants.
	 *  @return The confidence value of a certain joint. Between 0f and 1f. If jointType out of range: 0f */
	public float getJointConfidence (short jointType) {
		if (jointType >= 0 && jointType <= 14) 
			return jointConfidence[jointType];
		else
			return 0f;
	}
	/** The positions of the joints are evaluated with a certain confidence value. This method returns the confidence value for a certain joint. This method returns the unmirrored information, regardless of mirror therapy mode.
	 *  @param jointType The joint for which confidence value should be returned. Should be a short value corresponding to Skeleton constants.
	 *  @return The confidence value of a certain joint. Between 0f and 1f. If jointType out of range: 0f */
	public float getJointConfidenceUnmirrored (short jointType) {
		if (jointType >= 0 && jointType <= 14) 
			return jointConfidenceUnmirrored[jointType];
		else
			return 0f;
	}
	/** The orientations of the joints are evaluated with a certain confidence value. This method returns the orientation matrix 
	 *  @param jointType The joint for which confidence value should be returned. Should be a short value corresponding to Skeleton constants.
	 *  @return The orientation matrix of a certain joint. PMatrix3D. If jointType out of range: 0-Matrix */
	public PMatrix3D getJointOrientation (short jointType) {
		PMatrix3D returnValue = new PMatrix3D();
		if (jointType >= 0 && jointType <= 14) returnValue.set(jointOrientation[jointType]);
		return returnValue;
	}
	/** The orientations of the joints are evaluated with a certain confidence value. This method returns the orientation matrix. This method returns the unmirrored information, regardless of mirror therapy mode.
	 *  @param jointType The joint for which confidence value should be returned. Should be a short value corresponding to Skeleton constants.
	 *  @return The orientation matrix of a certain joint. PMatrix3D. If jointType out of range: 0-Matrix */
	public PMatrix3D getJointOrientationUnmirrored (short jointType) {
		PMatrix3D returnValue = new PMatrix3D();
		if (jointType >= 0 && jointType <= 14) returnValue.set(jointOrientationUnmirrored[jointType]);
		return returnValue;
	}
	/** The orientations of the joints are evaluated with a certain confidence value. This method returns the confidence value 
	 *  @param jointType The joint for which confidence value should be returned. Should be a short value corresponding to Skeleton constants.
	 *  @return The confidence value for the evaluated orientation of a certain joint. Between 0f and 1f. If jointType out of range: 0f */
	public float getJointOrientationConfidence (short jointType) {
		if (jointType >= 0 && jointType <= 14) 
			return jointOrientationConfidence[jointType];
		else
			return 0f;
	}
	/** The orientations of the joints are evaluated with a certain confidence value. This method returns the confidence value.  This method returns the unmirrored information, regardless of mirror therapy mode.
	 *  @param jointType The joint for which confidence value should be returned. Should be a short value corresponding to Skeleton constants.
	 *  @return The confidence value for the evaluated orientation of a certain joint. Between 0f and 1f. If jointType out of range: 0f */
	public float getJointOrientationConfidenceUnmirrored (short jointType) {
		if (jointType >= 0 && jointType <= 14) 
			return jointOrientationConfidenceUnmirrored[jointType];
		else
			return 0f;
	}
	/** This method returns the delta of the current joint position to the last joint position, i.e. the distance, which the joint moved during the last frame
	 *  @param jointType The joint for which confidence value should be returned. Should be a short value corresponding to Skeleton constants.
	 *  @return The distance between the current joint position and the last joint position. */
	public float getJointDelta (short jointType) {
		if (jointType >= 0 && jointType <= 14) 
			return jointDelta[jointType];
		else
			return 0f;
	}
	/** This method returns the delta of the current joint position to the last joint position, i.e. the distance, which the joint moved during the last frame. This method returns the unmirrored information, regardless of mirror therapy mode.
	 *  @param jointType The joint for which confidence value should be returned. Should be a short value corresponding to Skeleton constants.
	 *  @return The distance between the current joint position and the last joint position. */
	public float getJointDeltaUnmirrored (short jointType) {
		if (jointType >= 0 && jointType <= 14) 
			return jointDeltaUnmirrored[jointType];
		else
			return 0f;
	}
	/** returns the distance of the skeletons torso joint to the kinect
	 *  @return the distance in mm, magnitude of skeletons torso vector */
	public float distanceToKinect () {
		return joint[TORSO].mag();
	}

	
	// -----------------------------------------------------------------
	// ACCESS TO ARM VECTORS
	/** The vectors for lower and upper arms are calculated for convenience. 
	 *  @return The vector for the left upper arm */
	public PVector getLeftUpperArm() {
		PVector returnValue = new PVector();
		returnValue.set(lUpperArm);
		return returnValue;
	}
	/** The vectors for lower and upper arms are calculated for convenience. 
	 *  @return The vector for the left lower arm */
	public PVector getLeftLowerArm() {
		PVector returnValue = new PVector();
		returnValue.set(lLowerArm);
		return returnValue;
	}
	/** The vectors for lower and upper arms are calculated for convenience. 
	 *  @return The vector for the right upper arm */
	public PVector getRightUpperArm() {
		PVector returnValue = new PVector();
		returnValue.set(rUpperArm);
		return returnValue;
	}
	/** The vectors for lower and upper arms are calculated for convenience. 
	 *  @return The vector for the right lower arm */
	public PVector getRightLowerArm() {
		PVector returnValue = new PVector();
		returnValue.set(rLowerArm);
		return returnValue;
	}
	/** The vectors for the lower and upper arms are calculated for convenience.
	 *  @return The vector for the left upper arm in the local coordinate system.*/
	public PVector getLeftUpperArmLCS() {
		PVector returnValue = new PVector();
		returnValue.set(lUpperArmLCS);
		return returnValue;
	}
	/** The vectors for the lower and upper arms are calculated for convenience. 
	 *  @return The vector for the left lower arm in the local coordinate system.  */
	public PVector getRightUpperArmLCS() {
		PVector returnValue = new PVector();
		returnValue.set(rUpperArmLCS);
		return returnValue;
	}
	/** The vectors for the lower and upper arms are calculated for convenience. 
	 *  @return The vector for the right upper arm in the local coordinate system.  */
	public PVector getLeftLowerArmLCS() {
		PVector returnValue = new PVector();
		returnValue.set(lLowerArmLCS);
		return returnValue;
	}
	/** The vectors for the lower and upper arms are calculated for convenience. 
	 *  @return The vector for the right lower arm in the local coordinate system.  */
	public PVector getRightLowerArmLCS() {
		PVector returnValue = new PVector();
		returnValue.set(rLowerArmLCS);
		return returnValue;
	}
	/** The vectors for lower and upper arms are calculated for convenience. This method returns the unmirrored arm vectors.
	 *  @return The vector for the left upper arm */
	public PVector getLeftUpperArmUnmirrored() {
		PVector returnValue = new PVector();
		returnValue.set(lUpperArmUnmirrored);
		return returnValue;
	}
	/** The vectors for lower and upper arms are calculated for convenience.  This method returns the unmirrored arm vectors.
	 *  @return The vector for the left lower arm */
	public PVector getLeftLowerArmUnmirrored() {
		PVector returnValue = new PVector();
		returnValue.set(lLowerArmUnmirrored);
		return returnValue;
	}
	/** The vectors for lower and upper arms are calculated for convenience.  This method returns the unmirrored arm vectors.
	 *  @return The vector for the right upper arm */
	public PVector getRightUpperArmUnmirrored() {
		PVector returnValue = new PVector();
		returnValue.set(rUpperArmUnmirrored);
		return returnValue;
	}
	/** The vectors for lower and upper arms are calculated for convenience.  This method returns the unmirrored arm vectors.
	 *  @return The vector for the right lower arm */
	public PVector getRightLowerArmUnmirrored() {
		PVector returnValue = new PVector();
		returnValue.set(rLowerArmUnmirrored);
		return returnValue;
	}
	/** The vectors for the lower and upper arms are calculated for convenience. This method returns the unmirrored arm vectors.
	 *  @return The vector for the left upper arm in the local coordinate system.*/
	public PVector getLeftUpperArmLCSUnmirrored() {
		PVector returnValue = new PVector();
		returnValue.set(lUpperArmLCSUnmirrored);
		return returnValue;
	}
	/** The vectors for the lower and upper arms are calculated for convenience.  This method returns the unmirrored arm vectors.
	 *  @return The vector for the left lower arm in the local coordinate system.  */
	public PVector getRightUpperArmLCSUnmirrored() {
		PVector returnValue = new PVector();
		returnValue.set(rUpperArmLCSUnmirrored);
		return returnValue;
	}
	/** The vectors for the lower and upper arms are calculated for convenience.  This method returns the unmirrored arm vectors.
	 *  @return The vector for the right upper arm in the local coordinate system.  */
	public PVector getLeftLowerArmLCSUnmirrored() {
		PVector returnValue = new PVector();
		returnValue.set(lLowerArmLCSUnmirrored);
		return returnValue;
	}
	/** The vectors for the lower and upper arms are calculated for convenience.  This method returns the unmirrored arm vectors.
	 *  @return The vector for the right lower arm in the local coordinate system.  */
	public PVector getRightLowerArmLCSUnmirrored() {
		PVector returnValue = new PVector();
		returnValue.set(rLowerArmLCSUnmirrored);
		return returnValue;
	}
	
	
	// -----------------------------------------------------------------
	// ACCESS TO ANGLES OF BODY PARTY
	/** returns the angle between two limb-vectors
	 *  @param joint11 the joint the limb-vector1 points to
	 *  @param joint12 the joint the limb-vector1 origins in
	 *  @param joint21 the joint the limb-vector2 points to
	 *  @param joint22 the joint the limb-vector2 origins in
	 *  @return the angle, float between 0 and PI */
	public float angleBetween (short joint11, short joint12, short joint21, short joint22) {
		PVector axis1 = PVector.sub(joint[joint11],joint[joint12]);
		PVector axis2 = PVector.sub(joint[joint21],joint[joint22]);
		return PVector.angleBetween(axis1,axis2);
	}
	/** returns the angle between the limb-vector and local X axis (shoulder_l-shoulder_r)
	 *  @param joint11 the joint the limb-vector points to
	 *  @param joint12 the joint the limb-vector origins in
	 *  @return the angle, float between 0 and PI.*/
	public float angleToLocalXAxis (short joint11, short joint12) {
		PVector axis1 = PVector.sub(jointLCS[joint11],jointLCS[joint12]);
		return PVector.angleBetween(axis1,getOrientationX());
	}
	/** returns the angle between the limb-vector and local Y axis (neck-torso)
	 *  @param joint11 the joint the limb-vector points to
	 *  @param joint12 the joint the limb-vector origins in
	 *  @return the angle, float between 0 and PI.*/
	public float angleToLocalYAxis (short joint11, short joint12) {
		PVector axis1 = PVector.sub(jointLCS[joint11],jointLCS[joint12]);
		return PVector.angleBetween(axis1,getOrientationY()); 
	}
	/** returns the angle between the limb-vector and local Z axis (orthogonal on local x/y-plane)
	 *  @param joint11 the joint the limb-vector points to
	 *  @param joint12 the joint the limb-vector origins in
	 *  @return the angle, float between 0 and PI. */
	public float angleToLocalZAxis (short joint11, short joint12) {
		PVector axis1 = PVector.sub(jointLCS[joint11],jointLCS[joint12]);
		return PVector.angleBetween(axis1,getOrientationZ());
	}
	/** returns the angle between the limb-vector and the global X axis
	 *  @param joint11 the joint the limb-vector points to
	 *  @param joint12 the joint the limb-vector origins in
	 *  @return the angle, float between 0 and PI */
	public float angleToGlobalXAxis (short joint11, short joint12) {
		PVector axis1 = PVector.sub(joint[joint11],joint[joint12]);
		return PVector.angleBetween(axis1,new PVector(1,0,0));
	}
	/** returns the angle between the limb-vector and the global Y axis
	 *  @param joint11 the joint the limb-vector points to
	 *  @param joint12 the joint the limb-vector origins in
	 *  @return the angle, float between 0 and PI */
	public float angleToGlobalYAxis (short joint11, short joint12) {
		PVector axis1 = PVector.sub(joint[joint11],joint[joint12]);
		return PVector.angleBetween(axis1,new PVector(0,1,0));
	}
	/** returns the angle between the limb-vector and the global Z axis
	 *  @param joint11 the joint the limb-vector points to
	 *  @param joint12 the joint the limb-vector origins in
	 *  @return the angle, float between 0 and PI */
	public float angleToGlobalZAxis (short joint11, short joint12) {
		PVector axis1 = PVector.sub(joint[joint11],joint[joint12]);
		return PVector.angleBetween(axis1,new PVector(0,0,1));
	}
	/** returns the angle between the limb-vector and the normal vector of the sagittal body plane
	 *  @param joint11 the joint the limb-vector points to
	 *  @param joint12 the joint the limb-vector origins in
	 *  @return the angle, float between 0 and PI */
	public float angleToSagittalPlaneN0 (short joint11, short joint12) {
		PVector axis1 = PVector.sub(joint[joint11],joint[joint12]);
		return PVector.angleBetween(axis1,math.getSagittalPlane().n0);
	}
	/** returns the angle between the limb-vector and the normal vector of the frontal body plane
	 *  @param joint11 the joint the limb-vector points to
	 *  @param joint12 the joint the limb-vector origins in
	 *  @return the angle, float between 0 and PI */
	public float angleToFrontalPlaneN0 (short joint11, short joint12) {
		PVector axis1 = PVector.sub(joint[joint11],joint[joint12]);
		return PVector.angleBetween(axis1,math.getFrontalPlane().n0);
	}
	/** returns the angle between the limb-vector and the normal vector of the transversal body plane
	 *  @param joint11 the joint the limb-vector points to
	 *  @param joint12 the joint the limb-vector origins in
	 *  @return the angle, float between 0 and PI */
	public float angleToTransversalPlaneN0 (short joint11, short joint12) {
		PVector axis1 = PVector.sub(joint[joint11],joint[joint12]);
		return PVector.angleBetween(axis1,math.getTransversalPlane().n0);
	}
	/** The angle between the left upper Arm and the body axis. Is calculated in the local coordinate system!
	 *  @return The angle between the left upper Arm and the body axis.*/
	public float getAngleLeftUpperArm() {
		return PVector.angleBetween(lUpperArmLCS,getOrientationY());
	}
	/** The angle between the left lower Arm and the left upper arm.  Is calculated in the local coordinate system!
	 *  @return The angle between the left lower Arm and the left upper arm.*/
	public float getAngleLeftLowerArm() {
		return PVector.angleBetween(lLowerArmLCS,lUpperArmLCS);
	}
	/** The angle between the right upper Arm and the body axis.  Is calculated in the local coordinate system!
	 *  @return The angle between the right upper Arm and the body axis. */
	public float getAngleRightUpperArm() {
		return PVector.angleBetween(rUpperArmLCS,getOrientationY());
	}
	/** The angle between the right lower Arm and the right upper arm.  Is calculated in the local coordinate system!
	 *  @return The angle between the right lower Arm and the right upper arm. */
	public float getAngleRightLowerArm() {
		return PVector.angleBetween(rLowerArmLCS,rUpperArmLCS);
	}
	/** The angle between the left upper Arm and the body axis. Is calculated in the local coordinate system! This method uses the unmirrored arm vectors.
	 *  @return The angle between the left upper Arm and the body axis.*/
	public float getAngleLeftUpperArmUnmirrored() {
		return PVector.angleBetween(lUpperArmLCSUnmirrored,getOrientationY());
	}
	/** The angle between the left lower Arm and the left upper arm.  Is calculated in the local coordinate system! This method uses the unmirrored arm vectors.
	 *  @return The angle between the left lower Arm and the left upper arm.*/
	public float getAngleLeftLowerArmUnmirrored() {
		return PVector.angleBetween(lLowerArmLCSUnmirrored,lUpperArmLCSUnmirrored);
	}
	/** The angle between the right upper Arm and the body axis.  Is calculated in the local coordinate system! This method uses the unmirrored arm vectors.
	 *  @return The angle between the right upper Arm and the body axis. */
	public float getAngleRightUpperArmUnmirrored() {
		return PVector.angleBetween(rUpperArmLCSUnmirrored,getOrientationY());
	}
	/** The angle between the right lower Arm and the right upper arm.  Is calculated in the local coordinate system! This method uses the unmirrored arm vectors.
	 *  @return The angle between the right lower Arm and the right upper arm. */
	public float getAngleRightLowerArmUnmirrored() {
		return PVector.angleBetween(rLowerArmLCSUnmirrored,rUpperArmLCSUnmirrored);
	}
	
	// -----------------------------------------------------------------
	// ACCESS TO CLINICAL RELEVANT INFORMATION
	/** returns the orientation of a limb with respect to the frontal plane
	 *  @param joint11 the joint the limb-vector points to
	 *  @param joint12 the joint the limb-vector origins in
	 *  @return the orientation, short constant of skeleton (left_lateral, right_lateral, neutral) */
	public short getOrientationInFrontalPlane (short joint11, short joint12) {
		// +X-axis from torso->right
		if (jointLCS[joint11].x < jointLCS[joint12].x) {
			return LEFT_LATERAL;
		} else if (jointLCS[joint11].x > jointLCS[joint12].x) {
			return RIGHT_LATERAL;
		} else {
			return NEUTRAL;
		}
	}
	/** returns the orientation of a limb with respect to the sagittal plane
	 *  @param joint11 the joint the limb-vector points to
	 *  @param joint12 the joint the limb-vector origins in
	 *  @return the orientation, short constant of skeleton (posterior, anterior, neutral) */
	public short getOrientationInSagittalPlane (short joint11, short joint12) {
		// +Z-axis from torso->front
		if (jointLCS[joint11].z < jointLCS[joint12].z) {
			return POSTERIOR;
		} else if (jointLCS[joint11].z > jointLCS[joint12].z) {
			return ANTERIOR;
		} else {
			return NEUTRAL;
		}
	}
	/** returns the orientation of a limb with respect to the transversal plane
	 *  @param joint11 the joint the limb-vector points to
	 *  @param joint12 the joint the limb-vector origins in
	 *  @return the orientation, short constant of skeleton (inferior, superior, neutral) */
	public short getOrientationInTransversalPlane (short joint11, short joint12) {
		// +Y-axis from torso->head
		if (jointLCS[joint11].y < jointLCS[joint12].y) {
			return INFERIOR;
		} else if (jointLCS[joint11].y > jointLCS[joint12].y) {
			return SUPERIOR;
		} else {
			return NEUTRAL;
		}
	}
	/** returns the angle of the limb in the frontal plane to the negative y-axis: Abduction. 
	 *  Abduction is only defined for shoulder and hip-joints!!!
	 *  @param joint11 the joint the limb-vector points to
	 *  @param joint12 the joint the limb-vector origins in
	 *  @return the angle of the limb's abduction */
	public float getAbduction (short joint11, short joint12) {
		float abductionModifier = 1f;
		// Abduction is angle of the axis-components in frontal plane to negative body axis
		if ((joint11 == LEFT_ELBOW && joint12 == LEFT_SHOULDER) || 
			(joint11 == LEFT_KNEE && joint12 == LEFT_HIP)) {
			if (getOrientationInFrontalPlane(joint11,joint12) == RIGHT_LATERAL) {
				abductionModifier *= -1f;
			}
		} else if ((joint11 == RIGHT_ELBOW && joint12 == RIGHT_SHOULDER) || 
				(joint11 == RIGHT_KNEE && joint12 == RIGHT_HIP)) {
			if (getOrientationInFrontalPlane(joint11,joint12) == LEFT_LATERAL) {
				abductionModifier *= -1f;
			}
		} else {
			return 0f;
		}
		
		PVector axis1 = PVector.sub(jointLCS[joint11],jointLCS[joint12]);
		axis1.z = 0f; // use component vector of axis1 in frontal plane
		float angle = PVector.angleBetween(axis1,new PVector(0f,-1f,0f));
		return angle*=abductionModifier;
	}
	/** returns the angle of the limb in the frontal plane to the negative y-axis: Adduction. 
	 *  Adduction is only defined for shoulder and hip-joints!!!
	 *  @param joint11 the joint the limb-vector points to
	 *  @param joint12 the joint the limb-vector origins in
	 *  @return the angle of the limb's adduction */
	public float getAdduction (short joint11, short joint12) {
		float adductionModifier = 1f;
		// Adduction is angle of the axis-components in frontal plane to negative body axis
		if ((joint11 == LEFT_ELBOW && joint12 == LEFT_SHOULDER) || 
			(joint11 == LEFT_KNEE && joint12 == LEFT_HIP)) {
			if (getOrientationInFrontalPlane(joint11,joint12) == LEFT_LATERAL) {
				adductionModifier *= -1f;
			}
		} else if ((joint11 == RIGHT_ELBOW && joint12 == RIGHT_SHOULDER) || 
				(joint11 == RIGHT_KNEE && joint12 == RIGHT_HIP)) {
			if (getOrientationInFrontalPlane(joint11,joint12) == RIGHT_LATERAL) {
				adductionModifier *= -1f;
			}
		} else {
			return 0f;
		}
		
		PVector axis1 = PVector.sub(jointLCS[joint11],jointLCS[joint12]);
		axis1.z = 0f; // use only components of axis1 in frontal plane
		float angle = PVector.angleBetween(axis1,new PVector(0f,-1f,0f));
		
		return angle*=adductionModifier;
	}
	/** returns the angle of the limb in the sagittal plane to the negative y-axis: Anteversion. 
	 *  Anteversion is only defined for shoulder and hip-joints!!!
	 *  @param joint11 the joint the limb-vector points to
	 *  @param joint12 the joint the limb-vector origins in
	 *  @return the angle of the limb's Anteversion */
	public float getAnteversion (short joint11, short joint12) {
		float anteversionModifier = 1f;
		// Anteversion is angle of the axis-components in sagittal plane to negative body axis
		if ((joint11 == LEFT_ELBOW && joint12 == LEFT_SHOULDER) || 
			(joint11 == LEFT_KNEE && joint12 == LEFT_HIP) ||
			(joint11 == RIGHT_ELBOW && joint12 == RIGHT_SHOULDER) || 
			(joint11 == RIGHT_KNEE && joint12 == RIGHT_HIP)) {
			if (getOrientationInSagittalPlane(joint11,joint12) == POSTERIOR) {
				anteversionModifier *= -1f;
			}
		} else {
			return 0f;
		}
		
		PVector axis1 = PVector.sub(jointLCS[joint11],jointLCS[joint12]);
		axis1.x = 0f; // use component vector of axis1 in sagittal plane
		float angle = PVector.angleBetween(axis1,new PVector(0f,-1f,0f));
		
		return angle*=anteversionModifier;
	}
	/** returns the angle of the limb in the sagittal plane to the negative y-axis: Retroversion. 
	 *  Retroversion is only defined for shoulder and hip-joints!!!
	 *  @param joint11 the joint the limb-vector points to
	 *  @param joint12 the joint the limb-vector origins in
	 *  @return the angle of the limb's Retroversion */
	public float getRetroversion (short joint11, short joint12) {
		float retroversionModifier = 1f;
		// Retroversion is angle of the axis-components in sagittal plane to negative body axis
		if ((joint11 == LEFT_ELBOW && joint12 == LEFT_SHOULDER) || 
			(joint11 == LEFT_KNEE && joint12 == LEFT_HIP) ||
			(joint11 == RIGHT_ELBOW && joint12 == RIGHT_SHOULDER) || 
			(joint11 == RIGHT_KNEE && joint12 == RIGHT_HIP)) {
			if (getOrientationInSagittalPlane(joint11,joint12) == ANTERIOR) {
				retroversionModifier *= -1f;
			}
		} else {
			return 0f;
		}
		
		PVector axis1 = PVector.sub(jointLCS[joint11],jointLCS[joint12]);
		axis1.x = 0f; // use component vector of axis1 in sagittal plane
		float angle = PVector.angleBetween(axis1,new PVector(0f,-1f,0f));
		
		return angle*=retroversionModifier;
	}
	
	
	/** All planes are defined in HNF: r*n0-d = 0. Sagittal Plane is mirror plane!
	 *  @return the r vector of the sagittal body plane. */
	public PVector getRVectorSagittalPlane () {
		PVector returnValue = new PVector();
		returnValue.set(math.getSagittalPlane().r);
		return returnValue;
	}
	/** All planes are defined in HNF: r*n0-d = 0. Sagittal Plane is mirror plane!
	 *  @return the n0 vector of the sagittal body plane. */
	public PVector getN0VectorSagittalPlane () {
		PVector returnValue = new PVector();
		returnValue.set(math.getSagittalPlane().n0);
		return returnValue;
	}
	/** All planes are defined in HNF: r*n0-d = 0. Sagittal Plane is mirror plane!
	 *  @return the d value of the sagittal body plane. */
	public float getDValueSagittalPlane () {
		return math.getSagittalPlane().d;
	}
	/** All planes are defined in HNF: r*n0-d = 0.
	 *  @return the r vector of the frontal body plane. */
	public PVector getRVectorFrontalPlane () {
		PVector returnValue = new PVector();
		returnValue.set(math.getFrontalPlane().r);
		return returnValue;
	}
	/** All planes are defined in HNF: r*n0-d = 0.
	 *  @return the n0 vector of the frontal body plane. */
	public PVector getN0VectorFrontalPlane () {
		PVector returnValue = new PVector();
		returnValue.set(math.getFrontalPlane().n0);
		return returnValue;
	}
	/** All planes are defined in HNF: r*n0-d = 0.
	 *  @return the d value of the frontal body plane. */
	public float getDValueFrontalPlane () {
		return math.getFrontalPlane().d;
	}
	/** All planes are defined in HNF: r*n0-d = 0.
	 *  @return the r vector of the transversal body plane. */
	public PVector getRVectorTransversalPlane () {
		PVector returnValue = new PVector();
		returnValue.set(math.getTransversalPlane().r);
		return returnValue;
	}
	/** All planes are defined in HNF: r*n0-d = 0.
	 *  @return the n0 vector of the transversal body plane. */
	public PVector getN0VectorTransversalPlane () {
		PVector returnValue = new PVector();
		returnValue.set(math.getTransversalPlane().n0);
		return returnValue;
	}
	/** All planes are defined in HNF: r*n0-d = 0.
	 *  @return the d value of the transversal body plane. */
	public float getDValueTransversalPlane () {
		return math.getTransversalPlane().d;
	}
	/** returns the origin of the local coordsys. Equals Torso Vector.
	 *  @return the origin of the skeletons local coordinate system. */
	public PVector getOrigin () {
		PVector returnValue = new PVector();
		returnValue.set(math.getOrigin());
		return returnValue;
	}
	/** returns the local x coordinate vector.
	 *  @return the local x vector.*/
	public PVector getOrientationX () {
		PVector returnValue = new PVector();
		returnValue.set(math.getOrientationX());
		return returnValue;
	}
	/** returns the local x coordinate vector projected to the kinects projection plane.
	 *  @return the local vector projected to the kinects projection plane. */
	public PVector getOrientationXRealWorldProjective () {
		PVector projective = new PVector();
		kinect.convertRealWorldToProjective(this.getOrientationX(),projective);
		return projective;
	}
	/** returns the angle between the local x vector and the global x vector.
	 *  @return the angle, float between 0 and PI.  */
	public float getOrientationAlpha () {
		return PVector.angleBetween(this.getOrientationX(),new PVector(1,0,0)); 
	}
	/** returns the local y coordinate vector.
	 *  @return the local y vector. */
	public PVector getOrientationY () {
		PVector returnValue = new PVector();
		returnValue.set(math.getOrientationY());
		return returnValue;
	}
	/** returns the local y coordinate vector projected to the kinects projection plane.
	 *  @return the local vector projected to the kinects projection plane. */
	public PVector getOrientationYRealWorldProjective () {
		PVector projective = new PVector();
		kinect.convertRealWorldToProjective(getOrientationY(),projective);
		return projective;
	}
	/** returns the angle between the local y vector and the global y vector. 
	 *  @return the angle, float between 0 and PI.  */
	public float getOrientationBeta () {
		return PVector.angleBetween(getOrientationY(),new PVector(0,1,0)); 
	}
	/** returns the local z coordinate vector.
	 *  @return the local z vector.*/
	public PVector getOrientationZ () {
		PVector returnValue = new PVector();
		returnValue.set(math.getOrientationZ());
		return returnValue;
	}
	/** returns the local z coordinate vector projected to the kinects projection plane.
	 *  @return the local vector projected to the kinects projection plane.*/
	public PVector getOrientationZRealWorldProjective () {
		PVector projective = new PVector();
		kinect.convertRealWorldToProjective(getOrientationZ(),projective); 
		return projective;
	}
	/** returns the angle between the local z vector and the global z vector. 
	 *  @return the angle, float between 0 and PI. */
	public float getOrientationGamma () {
		return PVector.angleBetween(getOrientationZ(),new PVector(0,0,1)); 
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
	/** Returns the last gesture evaluated within _lookAtPastFrames, if calculation is activated and gesture was evaluated in last update cycle. See SkeletonGestures class for details
	 *  @param _lookAtPastFrames the number of past update cycles during which the gesture should have been recognized.
	 *  @return current upper body gesture. short, constants of SkeletonGestures class, NO_GESTURE if no gesture was recognized in the given past update cycles or gesture evaluation is switched off */
	public short getLastUpperBodyGesture (int _lookAtPastFrames) {
		if (gesture != null && gestureEvaluated)
			return gesture.getLastUpperBodyGesture(currentFrameCount-_lookAtPastFrames);
		else
			return SkeletonGesture.NO_GESTURE;
	}

	
	// -----------------------------------------------------------------
	// PRIVATE HELPER METHODS
	private void updateJointPositions () {
		PVector tempJoint = new PVector();
		
		jointConfidence[HEAD] = kinect.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_HEAD,tempJoint);
		jointDelta[HEAD] = (PVector.sub(joint[HEAD],tempJoint)).mag();
		joint[HEAD].set(tempJoint);
		jointConfidence[NECK] = kinect.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_NECK,tempJoint);
		jointDelta[NECK] = (PVector.sub(joint[NECK],tempJoint)).mag();
		joint[NECK].set(tempJoint);
		
		// OpenNI sets labels for left/right from the camera viewpoint. Here the anatomically correct labels are set.
		boolean setAnatomicallyCorrectLabels = true;
		if (setAnatomicallyCorrectLabels) {
			jointConfidence[LEFT_SHOULDER] = kinect.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_RIGHT_SHOULDER,tempJoint);
			jointDelta[LEFT_SHOULDER] = (PVector.sub(joint[LEFT_SHOULDER],tempJoint)).mag();
			joint[LEFT_SHOULDER].set(tempJoint);
			jointConfidence[RIGHT_SHOULDER] = kinect.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_LEFT_SHOULDER,tempJoint);
			jointDelta[RIGHT_SHOULDER] = (PVector.sub(joint[RIGHT_SHOULDER],tempJoint)).mag();
			joint[RIGHT_SHOULDER].set(tempJoint);
			jointConfidence[LEFT_ELBOW] = kinect.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_RIGHT_ELBOW,tempJoint);
			jointDelta[LEFT_ELBOW] = (PVector.sub(joint[LEFT_ELBOW],tempJoint)).mag();
			joint[LEFT_ELBOW].set(tempJoint);
			jointConfidence[LEFT_HAND] = kinect.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_RIGHT_HAND,tempJoint);
			jointDelta[LEFT_HAND] = (PVector.sub(joint[LEFT_HAND],tempJoint)).mag();
			joint[LEFT_HAND].set(tempJoint);
			jointConfidence[RIGHT_ELBOW] = kinect.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_LEFT_ELBOW,tempJoint);
			jointDelta[RIGHT_ELBOW] = (PVector.sub(joint[RIGHT_ELBOW],tempJoint)).mag();
			joint[RIGHT_ELBOW].set(tempJoint);
			jointConfidence[RIGHT_HAND] = kinect.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_LEFT_HAND,tempJoint);
			jointDelta[RIGHT_HAND] = (PVector.sub(joint[RIGHT_HAND],tempJoint)).mag();
			joint[RIGHT_HAND].set(tempJoint);
		} else {
			jointConfidence[LEFT_SHOULDER] = kinect.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_LEFT_SHOULDER,tempJoint);
			jointDelta[LEFT_SHOULDER] = (PVector.sub(joint[LEFT_SHOULDER],tempJoint)).mag();
			joint[LEFT_SHOULDER].set(tempJoint);
			jointConfidence[RIGHT_SHOULDER] = kinect.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_RIGHT_SHOULDER,tempJoint);
			jointDelta[RIGHT_SHOULDER] = (PVector.sub(joint[RIGHT_SHOULDER],tempJoint)).mag();
			joint[RIGHT_SHOULDER].set(tempJoint);
			jointConfidence[LEFT_ELBOW] = kinect.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_LEFT_ELBOW,tempJoint);
			jointDelta[LEFT_ELBOW] = (PVector.sub(joint[LEFT_ELBOW],tempJoint)).mag();
			joint[LEFT_ELBOW].set(tempJoint);
			jointConfidence[LEFT_HAND] = kinect.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_LEFT_HAND,tempJoint);
			jointDelta[LEFT_HAND] = (PVector.sub(joint[LEFT_HAND],tempJoint)).mag();
			joint[LEFT_HAND].set(tempJoint);
			jointConfidence[RIGHT_ELBOW] = kinect.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_RIGHT_ELBOW,tempJoint);
			jointDelta[RIGHT_ELBOW] = (PVector.sub(joint[RIGHT_ELBOW],tempJoint)).mag();
			joint[RIGHT_ELBOW].set(tempJoint);
			jointConfidence[RIGHT_HAND] = kinect.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_RIGHT_HAND,tempJoint);
			jointDelta[RIGHT_HAND] = (PVector.sub(joint[RIGHT_HAND],tempJoint)).mag();
			joint[RIGHT_HAND].set(tempJoint);
			
		}
		
		jointConfidence[TORSO] = kinect.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_TORSO,tempJoint);
		jointDelta[TORSO] = (PVector.sub(joint[TORSO],tempJoint)).mag();
		joint[TORSO].set(tempJoint);
		
		
		if (fullBodyTracking) {
			if (setAnatomicallyCorrectLabels) {
				jointConfidence[LEFT_HIP] = kinect.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_RIGHT_HIP,tempJoint);
				jointDelta[LEFT_HIP] = (PVector.sub(joint[LEFT_HIP],tempJoint)).mag();
				joint[LEFT_HIP].set(tempJoint);
				jointConfidence[LEFT_KNEE] = kinect.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_RIGHT_KNEE,tempJoint);
				jointDelta[LEFT_KNEE] = (PVector.sub(joint[LEFT_KNEE],tempJoint)).mag();
				joint[LEFT_KNEE].set(tempJoint);
				jointConfidence[LEFT_FOOT] = kinect.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_RIGHT_FOOT,tempJoint);
				jointDelta[LEFT_FOOT] = (PVector.sub(joint[LEFT_FOOT],tempJoint)).mag();
				joint[LEFT_FOOT].set(tempJoint);
				jointConfidence[RIGHT_HIP] = kinect.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_LEFT_HIP,tempJoint);
				jointDelta[RIGHT_HIP] = (PVector.sub(joint[RIGHT_HIP],tempJoint)).mag();
				joint[RIGHT_HIP].set(tempJoint);
				jointConfidence[RIGHT_KNEE] = kinect.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_LEFT_KNEE,tempJoint);
				jointDelta[RIGHT_KNEE] = (PVector.sub(joint[RIGHT_KNEE],tempJoint)).mag();
				joint[RIGHT_KNEE].set(tempJoint);
				jointConfidence[RIGHT_FOOT] = kinect.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_LEFT_FOOT,tempJoint);
				jointDelta[RIGHT_FOOT] = (PVector.sub(joint[RIGHT_FOOT],tempJoint)).mag();
				joint[RIGHT_FOOT].set(tempJoint);
			} else {
				jointConfidence[LEFT_HIP] = kinect.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_LEFT_HIP,tempJoint);
				jointDelta[LEFT_HIP] = (PVector.sub(joint[LEFT_HIP],tempJoint)).mag();
				joint[LEFT_HIP].set(tempJoint);
				jointConfidence[LEFT_KNEE] = kinect.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_LEFT_KNEE,tempJoint);
				jointDelta[LEFT_KNEE] = (PVector.sub(joint[LEFT_KNEE],tempJoint)).mag();
				joint[LEFT_KNEE].set(tempJoint);
				jointConfidence[LEFT_FOOT] = kinect.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_LEFT_FOOT,tempJoint);
				jointDelta[LEFT_FOOT] = (PVector.sub(joint[LEFT_FOOT],tempJoint)).mag();
				joint[LEFT_FOOT].set(tempJoint);
				jointConfidence[RIGHT_HIP] = kinect.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_RIGHT_HIP,tempJoint);
				jointDelta[RIGHT_HIP] = (PVector.sub(joint[RIGHT_HIP],tempJoint)).mag();
				joint[RIGHT_HIP].set(tempJoint);
				jointConfidence[RIGHT_KNEE] = kinect.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_RIGHT_KNEE,tempJoint);
				jointDelta[RIGHT_KNEE] = (PVector.sub(joint[RIGHT_KNEE],tempJoint)).mag();
				joint[RIGHT_KNEE].set(tempJoint);
				jointConfidence[RIGHT_FOOT] = kinect.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_RIGHT_FOOT,tempJoint);
				jointDelta[RIGHT_FOOT] = (PVector.sub(joint[RIGHT_FOOT],tempJoint)).mag();
				joint[RIGHT_FOOT].set(tempJoint);
			}
		}
		lUpperArm = PVector.sub(joint[LEFT_ELBOW],joint[LEFT_SHOULDER]);
		rUpperArm = PVector.sub(joint[RIGHT_ELBOW],joint[RIGHT_SHOULDER]);
		lLowerArm = PVector.sub(joint[LEFT_HAND],joint[LEFT_ELBOW]);
		rLowerArm = PVector.sub(joint[RIGHT_HAND],joint[RIGHT_ELBOW]);
		
		// store original joint information before mirroring takes place on data storage.
		for (int i=0; i<15; i++) {
			jointUnmirrored[i] = joint[i];
			jointDeltaUnmirrored[i] = jointDelta[i];
			jointConfidenceUnmirrored[i] = jointConfidence[i];
		}
		lUpperArmUnmirrored = lUpperArm;
		rUpperArmUnmirrored = rUpperArm;
		lLowerArmUnmirrored = lLowerArm;
		rLowerArmUnmirrored = rLowerArm;
		
	}
	private void updateJointOrientations () {
		jointOrientationConfidence[HEAD] = kinect.getJointOrientationSkeleton(userId,SimpleOpenNI.SKEL_HEAD,jointOrientation[HEAD]);
		jointOrientationConfidence[NECK] = kinect.getJointOrientationSkeleton(userId,SimpleOpenNI.SKEL_NECK,jointOrientation[NECK]);
		jointOrientationConfidence[LEFT_SHOULDER] = kinect.getJointOrientationSkeleton(userId,SimpleOpenNI.SKEL_LEFT_SHOULDER,jointOrientation[LEFT_SHOULDER]);
		jointOrientationConfidence[RIGHT_SHOULDER] = kinect.getJointOrientationSkeleton(userId,SimpleOpenNI.SKEL_RIGHT_SHOULDER,jointOrientation[RIGHT_SHOULDER]);
		jointOrientationConfidence[TORSO] = kinect.getJointOrientationSkeleton(userId,SimpleOpenNI.SKEL_TORSO,jointOrientation[TORSO]);
		jointOrientationConfidence[LEFT_ELBOW] = kinect.getJointOrientationSkeleton(userId,SimpleOpenNI.SKEL_LEFT_ELBOW,jointOrientation[LEFT_ELBOW]);
		jointOrientationConfidence[LEFT_HAND] = kinect.getJointOrientationSkeleton(userId,SimpleOpenNI.SKEL_LEFT_HAND,jointOrientation[LEFT_HAND]);
		jointOrientationConfidence[RIGHT_ELBOW] = kinect.getJointOrientationSkeleton(userId,SimpleOpenNI.SKEL_RIGHT_ELBOW,jointOrientation[RIGHT_ELBOW]);
		jointOrientationConfidence[RIGHT_HAND] = kinect.getJointOrientationSkeleton(userId,SimpleOpenNI.SKEL_RIGHT_HAND,jointOrientation[RIGHT_HAND]);
		if (fullBodyTracking) {
			jointOrientationConfidence[LEFT_HIP] = kinect.getJointOrientationSkeleton(userId,SimpleOpenNI.SKEL_LEFT_HIP,jointOrientation[LEFT_HIP]);
			jointOrientationConfidence[LEFT_KNEE] = kinect.getJointOrientationSkeleton(userId,SimpleOpenNI.SKEL_LEFT_KNEE,jointOrientation[LEFT_KNEE]);
			jointOrientationConfidence[LEFT_FOOT] = kinect.getJointOrientationSkeleton(userId,SimpleOpenNI.SKEL_LEFT_FOOT,jointOrientation[LEFT_FOOT]);
			jointOrientationConfidence[RIGHT_HIP] = kinect.getJointOrientationSkeleton(userId,SimpleOpenNI.SKEL_RIGHT_HIP,jointOrientation[RIGHT_HIP]);
			jointOrientationConfidence[RIGHT_KNEE] = kinect.getJointOrientationSkeleton(userId,SimpleOpenNI.SKEL_RIGHT_KNEE,jointOrientation[RIGHT_KNEE]);
			jointOrientationConfidence[RIGHT_FOOT] = kinect.getJointOrientationSkeleton(userId,SimpleOpenNI.SKEL_RIGHT_FOOT,jointOrientation[RIGHT_FOOT]);
		}
		
		// store original joint information before mirroring takes place on data storage.
		for (int i=0; i<15; i++) {
			jointOrientationUnmirrored[i] = jointOrientation[i];
			jointOrientationConfidenceUnmirrored[i] = jointOrientationConfidence[i];
		}
	}
	
	private void updateMirroredJointPositions () {
		switch (mirrorTherapy) {
			case MIRROR_THERAPY_LEFT:
				// mirror left elbow to right elbow
				joint[RIGHT_ELBOW].set(math.mirrorJointVector(joint[LEFT_ELBOW]));
				jointDelta[RIGHT_ELBOW] = jointDelta[LEFT_ELBOW];
				jointConfidence[RIGHT_ELBOW] = jointConfidence[LEFT_ELBOW];
				// mirror left hand to right hand
				joint[RIGHT_HAND].set(math.mirrorJointVector(joint[LEFT_HAND]));
				jointDelta[RIGHT_HAND] = jointDelta[LEFT_HAND];
				jointConfidence[RIGHT_HAND] = jointConfidence[LEFT_HAND];
				// update arm vectors
				rUpperArm = PVector.sub(joint[RIGHT_ELBOW],joint[RIGHT_SHOULDER]);
				rLowerArm = PVector.sub(joint[RIGHT_HAND],joint[RIGHT_ELBOW]);
				break;
			case MIRROR_THERAPY_RIGHT:
				// mirror right elbow to left elbow
				joint[LEFT_ELBOW].set(math.mirrorJointVector(joint[RIGHT_ELBOW]));
				jointDelta[LEFT_ELBOW] = jointDelta[RIGHT_ELBOW];
				jointConfidence[LEFT_ELBOW] = jointConfidence[RIGHT_ELBOW];
				// mirror right hand to left hand
				joint[LEFT_HAND].set(math.mirrorJointVector(joint[RIGHT_HAND]));
				jointDelta[LEFT_HAND] = jointDelta[RIGHT_HAND];
				jointConfidence[LEFT_HAND] = jointConfidence[RIGHT_HAND];
				// update arm vectors
				lUpperArm = PVector.sub(joint[LEFT_ELBOW],joint[LEFT_SHOULDER]);
				lLowerArm = PVector.sub(joint[LEFT_HAND],joint[LEFT_ELBOW]);
				break;
		}
	}
	private void updateMirroredJointOrientations () {
		switch (mirrorTherapy) {
			case MIRROR_THERAPY_LEFT:
				// mirror orientation of left shoulder to right shoulder
				jointOrientation[RIGHT_SHOULDER].set(math.mirrorOrientationMatrix(jointOrientation[LEFT_SHOULDER]));
				jointOrientationConfidence[RIGHT_SHOULDER] = jointOrientationConfidence[LEFT_SHOULDER];
				// mirror orientation of left elbow to right elbow
				jointOrientation[RIGHT_ELBOW].set(math.mirrorOrientationMatrix(jointOrientation[LEFT_ELBOW]));
				jointOrientationConfidence[RIGHT_ELBOW] = jointOrientationConfidence[LEFT_ELBOW];
				// mirror orientation of left hand to right hand
				jointOrientation[RIGHT_HAND].set(math.mirrorOrientationMatrix(jointOrientation[LEFT_HAND]));
				jointOrientationConfidence[RIGHT_HAND] = jointOrientationConfidence[LEFT_HAND];
				break;
			case MIRROR_THERAPY_RIGHT:
				// mirror orientation of right  shoulder to left shoulder
				jointOrientation[LEFT_SHOULDER].set(math.mirrorOrientationMatrix(jointOrientation[RIGHT_SHOULDER]));
				jointOrientationConfidence[LEFT_SHOULDER] = jointOrientationConfidence[RIGHT_SHOULDER];
				// mirror orientation of right  elbow to left elbow
				jointOrientation[LEFT_ELBOW].set(math.mirrorOrientationMatrix(jointOrientation[RIGHT_ELBOW]));
				jointOrientationConfidence[LEFT_ELBOW] = jointOrientationConfidence[RIGHT_ELBOW];
				// mirror orientation of right  hand to left hand
				jointOrientation[LEFT_HAND].set(math.mirrorOrientationMatrix(jointOrientation[RIGHT_HAND]));
				jointOrientationConfidence[LEFT_HAND] = jointOrientationConfidence[RIGHT_HAND];
				break;
		}	
	}
	private void transformToLCS () {
		jointLCS[HEAD] = math.getJointLCS(joint[HEAD]);
		jointLCS[NECK] = math.getJointLCS(joint[NECK]);
		jointLCS[TORSO] = math.getJointLCS(joint[TORSO]);
		jointLCS[LEFT_SHOULDER] = math.getJointLCS(joint[LEFT_SHOULDER]);
		jointLCS[RIGHT_SHOULDER] = math.getJointLCS(joint[RIGHT_SHOULDER]);
		jointLCS[LEFT_ELBOW] = math.getJointLCS(joint[LEFT_ELBOW]);
		jointLCS[RIGHT_ELBOW] = math.getJointLCS(joint[RIGHT_ELBOW]);
		jointLCS[LEFT_HAND] = math.getJointLCS(joint[LEFT_HAND]);
		jointLCS[RIGHT_HAND] = math.getJointLCS(joint[RIGHT_HAND]);
		if (fullBodyTracking) {
			jointLCS[LEFT_HIP] = math.getJointLCS(joint[LEFT_HIP]);
			jointLCS[LEFT_KNEE] = math.getJointLCS(joint[LEFT_KNEE]);
			jointLCS[LEFT_FOOT] = math.getJointLCS(joint[LEFT_FOOT]);
			jointLCS[RIGHT_HIP] = math.getJointLCS(joint[RIGHT_HIP]);
			jointLCS[RIGHT_KNEE] = math.getJointLCS(joint[RIGHT_KNEE]);
			jointLCS[RIGHT_FOOT] = math.getJointLCS(joint[RIGHT_FOOT]);
		}
		lUpperArmLCS = PVector.sub(jointLCS[LEFT_ELBOW],jointLCS[LEFT_SHOULDER]);
		rUpperArmLCS = PVector.sub(jointLCS[RIGHT_ELBOW],jointLCS[RIGHT_SHOULDER]);
		lLowerArmLCS = PVector.sub(jointLCS[LEFT_HAND],jointLCS[LEFT_ELBOW]);
		rLowerArmLCS = PVector.sub(jointLCS[RIGHT_HAND],jointLCS[RIGHT_ELBOW]);

		lUpperArmLCSUnmirrored = lUpperArmLCS;
		rUpperArmLCSUnmirrored = rUpperArmLCS;
		lLowerArmLCSUnmirrored = lLowerArmLCS;
		rLowerArmLCSUnmirrored = rLowerArmLCS;
		
		// calculate lcs for original joints
		for (int i=0; i<15; i++) {
			if (mirrorTherapy == MIRROR_THERAPY_LEFT && (i == RIGHT_ELBOW || i == RIGHT_HAND)) {
				jointLCSUnmirrored[i] = math.getJointLCS(jointUnmirrored[i]);
			} else if (mirrorTherapy == MIRROR_THERAPY_RIGHT && (i == LEFT_ELBOW || i == LEFT_HAND)) {
				jointLCSUnmirrored[i] = math.getJointLCS(jointUnmirrored[i]);
			} else {
				jointLCSUnmirrored[i] = jointLCS[i];
			}
		}
		if (mirrorTherapy == MIRROR_THERAPY_RIGHT) {
			lUpperArmLCSUnmirrored = PVector.sub(jointLCSUnmirrored[LEFT_ELBOW],jointLCSUnmirrored[LEFT_SHOULDER]);
			lLowerArmLCSUnmirrored = PVector.sub(jointLCSUnmirrored[LEFT_HAND],jointLCSUnmirrored[LEFT_ELBOW]);
		} else if (mirrorTherapy == MIRROR_THERAPY_LEFT) {
			rUpperArmLCSUnmirrored = PVector.sub(jointLCSUnmirrored[RIGHT_ELBOW],jointLCSUnmirrored[RIGHT_SHOULDER]);
			rLowerArmLCSUnmirrored = PVector.sub(jointLCSUnmirrored[RIGHT_HAND],jointLCSUnmirrored[RIGHT_ELBOW]);
		}
	}
}
	