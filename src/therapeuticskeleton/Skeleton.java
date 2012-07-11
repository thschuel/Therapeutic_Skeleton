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
	
	// shape of upper body joints, see doc/articulated_posture.txt
	public static final short NO_SHAPE = 0;
	public static final short V_SHAPE = 1;
	public static final short A_SHAPE = 2;
	public static final short U_SHAPE = 3;
	public static final short N_SHAPE = 4;
	public static final short M_SHAPE = 5;
	public static final short W_SHAPE = 6;
	public static final short O_SHAPE = 7;
	public static final short I_SHAPE = 8;
	public short activePostureShape = NO_SHAPE;
	
	// The interface to talk to kinect
	protected SimpleOpenNI kinect;
	
	// stores skeleton Points in 3d Space, global coordsys
	protected PVector[] skeletonPoints = new PVector[15]; 
	protected float[] confidenceSkeletonPoints = new float[15];
	// stores skeleton Points in 3d Space, local coordsys (neck is origin)
	protected PVector[] skeletonPointsLocal = new PVector[15]; 
	protected PVector origin;
	protected PVector orientationX, orientationY, orientationZ;
	protected PMatrix3D transformCoordSys;
	protected PMatrix3D transformCoordSysInv;
	// fast access to local hand, elbow and shoulder vectors for posture evaluation
	protected PVector localLHand, localRHand, localLElbow, localRElbow, localLShoulder, localRShoulder;
	
	// stores joint orientation
	protected PMatrix3D[] jointOrientations = new PMatrix3D[15];
	protected float[] confidenceJointOrientations = new float[15];
	
	// calculation of mirror plane
	private PVector[] bodyPoints = new PVector[7]; // stores body points of skeleton 
	private PVector	rMP = new PVector(); // MirrorPlane in HNF: r*n0-d=0
	private PVector	n0MP = new PVector();
	private float dMP = 0.0f;
	
	// setup variables
	protected boolean localCoordSys = true;
	protected boolean fullBodyTracking = false;
	protected short mirrorTherapy = MIRROR_THERAPY_OFF;
	
	// controls state of skeleton
	public boolean isUpdated = false;
	public boolean mirrorPlaneCalculated = false;
	public boolean localCoordSysCalculated = false;
	
	// skeleton of user
	public int userId;
	
	public Skeleton (SimpleOpenNI _kinect, int _userId, boolean _fullBodyTracking, boolean _localCoordSys, short _mirrorTherapy) {
		kinect = _kinect;
		userId = _userId;
		fullBodyTracking = _fullBodyTracking;
		localCoordSys = _localCoordSys;
		if (_mirrorTherapy >= MIRROR_THERAPY_OFF && _mirrorTherapy <= MIRROR_THERAPY_RIGHT) 
			mirrorTherapy = _mirrorTherapy;
		for (int i=0; i<15; i++){
			skeletonPoints[i] = new PVector();
			skeletonPointsLocal[i] = new PVector();
			jointOrientations[i] = new PMatrix3D();
		}
		for (int i=0; i<7; i++){
			bodyPoints[i] = new PVector();
		}
	}
	
	public void update () {
		isUpdated = false;
		localCoordSysCalculated = false;
		mirrorPlaneCalculated = false;
		
		updateJointPositions();
		updateJointOrientations();
		
		if (mirrorTherapy != MIRROR_THERAPY_OFF) {
			calculateMirrorPlane();
			updateMirroredJointPositions();
			updateMirroredJointOrientations();
		}
		if (localCoordSys) {
			calculateLocalCoordSys();
			transformToLocalCoordSys();
		}
		
		isUpdated = true;
	}
	

	// -----------------------------------------------------------------
	// methods to communicate
	public void setMirrorTherapy (short _mirrorTherapy) {
		if (_mirrorTherapy >= MIRROR_THERAPY_OFF && _mirrorTherapy <= MIRROR_THERAPY_RIGHT) 
			mirrorTherapy = _mirrorTherapy;
		else
			mirrorTherapy = MIRROR_THERAPY_OFF;
	}
	public short getMirrorTherapy () {
		return mirrorTherapy;
	}
	
	public void setFullBodyTracking (boolean _fullBodyTracking) {
		fullBodyTracking = _fullBodyTracking;
	}
	public boolean getFullBodyTracking () {
		return fullBodyTracking;
	}
	
	public void setCalculateLocalCoordSys (boolean _localCoordSys) {
		localCoordSys = _localCoordSys;
	}
	public boolean getCalculateLocalCoordSys () {
		return localCoordSys;
	}
	
	public PVector getJoint (short jointType) {
		if (jointType >= 0 && jointType <= 14) 
			return skeletonPoints[jointType];
		else
			return new PVector();
	}
	
	public PVector getJointLocalCoordSys (short jointType) {
		if (jointType >= 0 && jointType <= 14 && localCoordSysCalculated) 
			return skeletonPointsLocal[jointType];
		else
			return new PVector();
	}
	
	public float getConfidenceJoint (short jointType) {
		if (jointType >= 0 && jointType <= 14) 
			return confidenceJointOrientations[jointType];
		else
			return 0f;
	}
	
	public PMatrix3D getJointOrientation (short jointType) {
		if (jointType >= 0 && jointType <= 14) 
			return jointOrientations[jointType];
		else
			return new PMatrix3D();
	}
	
	public float getConfidenceJointOrientation (short jointType) {
		if (jointType >= 0 && jointType <= 14) 
			return confidenceJointOrientations[jointType];
		else
			return 0f;
	}
	
	public PVector getRVectorMirrorPlane () {
		if (mirrorPlaneCalculated)
			return rMP;
		else
			return new PVector();
	}
	
	public PVector getN0VectorMirrorPlane () {
		if (mirrorPlaneCalculated)
			return n0MP;
		else
			return new PVector();
	}
	
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
		PVector axis1 = PVector.sub(skeletonPoints[joint11],skeletonPoints[joint12]);
		PVector axis2 = PVector.sub(skeletonPoints[joint21],skeletonPoints[joint22]);
		return PVector.angleBetween(axis1,axis2);
	}
	
	/** returns the angle between the limb-vector and local X axis
	 *  @param joint11 the joint the limb-vector points to
	 *  @param joint12 the joint the limb-vector origins in
	 *  @return the angle, float between 0 and PI */
	public float angleToLocalXAxis (short joint11, short joint12) {
		PVector axis1 = PVector.sub(skeletonPointsLocal[joint11],skeletonPointsLocal[joint12]);
		return PVector.angleBetween(axis1,orientationX);
	}
	/** returns the angle between the limb-vector and local Y axis
	 *  @param joint11 the joint the limb-vector points to
	 *  @param joint12 the joint the limb-vector origins in
	 *  @return the angle, float between 0 and PI */
	public float angleToLocalYAxis (short joint11, short joint12) {
		PVector axis1 = PVector.sub(skeletonPointsLocal[joint11],skeletonPointsLocal[joint12]);
		return PVector.angleBetween(axis1,orientationY);
	}
	/** returns the angle between the limb-vector and local Z axis
	 *  @param joint11 the joint the limb-vector points to
	 *  @param joint12 the joint the limb-vector origins in
	 *  @return the angle, float between 0 and PI */
	public float angleToLocalZAxis (short joint11, short joint12) {
		PVector axis1 = PVector.sub(skeletonPointsLocal[joint11],skeletonPointsLocal[joint12]);
		return PVector.angleBetween(axis1,orientationZ);
	}

	/** returns the angle between the limb-vector and the global X axis
	 *  @param joint11 the joint the limb-vector points to
	 *  @param joint12 the joint the limb-vector origins in
	 *  @return the angle, float between 0 and PI */
	public float angleToXAxis (short joint11, short joint12) {
		PVector axis1 = PVector.sub(skeletonPoints[joint11],skeletonPoints[joint12]);
		return PVector.angleBetween(axis1,new PVector(1,0,0));
	}
	/** returns the angle between the limb-vector and the global Y axis
	 *  @param joint11 the joint the limb-vector points to
	 *  @param joint12 the joint the limb-vector origins in
	 *  @return the angle, float between 0 and PI */
	public float angleToYAxis (short joint11, short joint12) {
		PVector axis1 = PVector.sub(skeletonPoints[joint11],skeletonPoints[joint12]);
		return PVector.angleBetween(axis1,new PVector(0,1,0));
	}
	/** returns the angle between the limb-vector and the global Z axis
	 *  @param joint11 the joint the limb-vector points to
	 *  @param joint12 the joint the limb-vector origins in
	 *  @return the angle, float between 0 and PI */
	public float angleToZAxis (short joint11, short joint12) {
		PVector axis1 = PVector.sub(skeletonPoints[joint11],skeletonPoints[joint12]);
		return PVector.angleBetween(axis1,new PVector(0,0,1));
	}

	/** returns the distance of the skeletons torso joint to the kinect
	 *  @return the distance in mm, magnitude of skeletons torso vector */
	public float distanceToKinect () {
		return skeletonPoints[Skeleton.TORSO].mag();
	}
	
	// -----------------------------------------------------------------
	// methods to calculate body posture
	// evaluate upper body joints according to articulated postures defined in doc/articulated_posture.txt
	public short getUpperJointPosture () {
		if (isUpdated && localCoordSysCalculated) {
			localLHand = skeletonPoints[Skeleton.LEFT_HAND];
			localRHand = skeletonPoints[Skeleton.RIGHT_HAND];
			localLElbow = skeletonPoints[Skeleton.LEFT_ELBOW];
			localRElbow = skeletonPoints[Skeleton.RIGHT_ELBOW];
			localLShoulder = skeletonPoints[Skeleton.LEFT_SHOULDER];
			localRShoulder = skeletonPoints[Skeleton.RIGHT_SHOULDER];
			if (evaluateVShape()) activePostureShape = V_SHAPE;
			else if (evaluateAShape()) activePostureShape = A_SHAPE;
			else if (evaluateUShape()) activePostureShape = U_SHAPE;
			else if (evaluateNShape()) activePostureShape = N_SHAPE;
			else if (evaluateMShape()) activePostureShape = M_SHAPE;
			else if (evaluateWShape()) activePostureShape = W_SHAPE;
			else if (evaluateOShape()) activePostureShape = O_SHAPE;
			else if (evaluateIShape()) activePostureShape = I_SHAPE;
			else activePostureShape = NO_SHAPE;
			return activePostureShape;
		} else {
			activePostureShape = NO_SHAPE;
			return activePostureShape;
		}
	}

	private boolean evaluateIShape() {
		// TODO Auto-generated method stub
		return false;
	}

	private boolean evaluateOShape() {
		PVector lElbowShoulder = PVector.sub(localLElbow,localLShoulder);
		PVector rElbowShoulder = PVector.sub(localRElbow,localRShoulder);
		PVector lHandElbow = PVector.sub(localLHand,localLElbow);
		PVector rHandElbow = PVector.sub(localRHand,localRElbow);
		PVector rHandlHand = PVector.sub(localRHand,localLHand);
		float angleL = PVector.angleBetween(lElbowShoulder,lHandElbow);
		float angleR = PVector.angleBetween(rElbowShoulder,rHandElbow);
		if (valueBetween(angleL,PConstants.PI/5,2*PConstants.PI/3) && valueBetween(angleR,PConstants.PI/4,PConstants.PI/2)) { // arms form an angle between 45 and 90 degree
			if (valueBetween(rHandlHand.mag(),0,100)) {
				return true;
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
		// TODO Auto-generated method stub
		return false;
	}

	private boolean evaluateUShape() {
		// TODO Auto-generated method stub
		return false;
	}

	private boolean evaluateAShape() {
		// TODO Auto-generated method stub
		return false;
	}

	private boolean evaluateVShape() {
		PVector lElbowShoulder = PVector.sub(localLElbow,localLShoulder);
		PVector rElbowShoulder = PVector.sub(localRElbow,localRShoulder);
		PVector lHandElbow = PVector.sub(localLHand,localLElbow);
		PVector rHandElbow = PVector.sub(localRHand,localRElbow);
		float angleL = PVector.angleBetween(lElbowShoulder,lHandElbow);
		float angleR = PVector.angleBetween(rElbowShoulder,rHandElbow);
		float angleVShape = PVector.angleBetween(lElbowShoulder, rElbowShoulder);
		if (valueBetween(angleL,0,PConstants.PI/8) && valueBetween(angleR,0,PConstants.PI/8)) { // arms form a straight line
			if (valueBetween(angleVShape,PConstants.PI/4,PConstants.PI/2)) { // arms angle between 45 and 90 degree
				return true;
			}
		}
		return false;
	}

	// -----------------------------------------------------------------
	// maths
	// evaluate local coord sys
	// origin: neck
	// orientation: 
	// +x==right_shoulder-left_shoulder, 
	// -y==orthogonal on +x and pointing to torso
	// +z==cross product of x and y
	private void calculateLocalCoordSys () {
		localCoordSysCalculated = false;
		origin = skeletonPoints[Skeleton.NECK];
		
		// *** calculating local coordSys
		// +x-axis==right_shoulder-left_shoulder, 
		orientationX = PVector.sub(skeletonPoints[Skeleton.RIGHT_SHOULDER],skeletonPoints[Skeleton.LEFT_SHOULDER]);
		// +y==orthogonal to +x-axis, pointing from torso to x-axis. 
		// task: find point on orientationX
		// - the plane that contains torso and has orientationX as normal vector is defined as: 
		// - x1*orientationX.x+x2*orientationX.y+x3*orientationX.z=torso(dot)orientationX
		// - straight line defined by orientationX: [x] = left_shoulder+lambda*orientationX
		// - find lambda of crosspoint of that line with the plane: insert straight line as X into plane
		// - use lambda in straight line equation to get crosspoint
		// - +y is crosspoint-torso
		float lambda = skeletonPoints[Skeleton.TORSO].dot(orientationX);
		lambda -= orientationX.dot(skeletonPoints[Skeleton.LEFT_SHOULDER]); 
		lambda /= orientationX.dot(orientationX);
		PVector crossPoint = PVector.add(skeletonPoints[Skeleton.LEFT_SHOULDER],PVector.mult(orientationX,lambda));
		orientationY = PVector.sub(crossPoint,skeletonPoints[Skeleton.TORSO]);
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
		r = skeletonPoints[Skeleton.TORSO];
		// n0 is cross product of two vectors in the plane
		PVector temp1 = PVector.sub(skeletonPoints[Skeleton.LEFT_SHOULDER],r);
		PVector temp2 = PVector.sub(skeletonPoints[Skeleton.RIGHT_SHOULDER],r);
		n0 = temp1.cross(temp2);
		n0.normalize();
		// mirrorPlane is orthogonal to body plane and contains the line between torso and neck
		// calculate mirrorPlane in HNF: r*n0-d = 0
		rMP = r; // r is always set to position of the torso
		n0MP = n0.cross(PVector.sub(skeletonPoints[Skeleton.NECK],skeletonPoints[Skeleton.TORSO]));
		n0MP.normalize();
		dMP = PVector.dot(rMP,n0MP);
		mirrorPlaneCalculated = true;
	}

	private boolean valueBetween (float val, float lowerBound, float upperBound) {
			return (val >= lowerBound && val <= upperBound);
	}
	
	// -----------------------------------------------------------------
	// update skeleton methods
	private void updateJointPositions () {
		confidenceSkeletonPoints[Skeleton.HEAD] = kinect.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_HEAD,skeletonPoints[Skeleton.HEAD]);
		confidenceSkeletonPoints[Skeleton.NECK] = kinect.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_NECK,skeletonPoints[Skeleton.NECK]);
		confidenceSkeletonPoints[Skeleton.LEFT_SHOULDER] = kinect.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_LEFT_SHOULDER,skeletonPoints[Skeleton.LEFT_SHOULDER]);
		confidenceSkeletonPoints[Skeleton.RIGHT_SHOULDER] = kinect.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_RIGHT_SHOULDER,skeletonPoints[Skeleton.RIGHT_SHOULDER]);
		confidenceSkeletonPoints[Skeleton.TORSO] = kinect.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_TORSO,skeletonPoints[Skeleton.TORSO]);
		confidenceSkeletonPoints[Skeleton.LEFT_ELBOW] = kinect.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_LEFT_ELBOW,skeletonPoints[Skeleton.LEFT_ELBOW]);
		confidenceSkeletonPoints[Skeleton.LEFT_HAND] = kinect.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_LEFT_HAND,skeletonPoints[Skeleton.LEFT_HAND]);
		confidenceSkeletonPoints[Skeleton.RIGHT_ELBOW] = kinect.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_RIGHT_ELBOW,skeletonPoints[Skeleton.RIGHT_ELBOW]);
		confidenceSkeletonPoints[Skeleton.RIGHT_HAND] = kinect.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_RIGHT_HAND,skeletonPoints[Skeleton.RIGHT_HAND]);
		if (fullBodyTracking) {
			confidenceSkeletonPoints[Skeleton.LEFT_HIP] = kinect.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_LEFT_HIP,skeletonPoints[Skeleton.LEFT_HIP]);
			confidenceSkeletonPoints[Skeleton.LEFT_KNEE] = kinect.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_LEFT_KNEE,skeletonPoints[Skeleton.LEFT_KNEE]);
			confidenceSkeletonPoints[Skeleton.LEFT_FOOT] = kinect.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_LEFT_FOOT,skeletonPoints[Skeleton.LEFT_FOOT]);
			confidenceSkeletonPoints[Skeleton.RIGHT_HIP] = kinect.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_RIGHT_HIP,skeletonPoints[Skeleton.RIGHT_HIP]);
			confidenceSkeletonPoints[Skeleton.RIGHT_KNEE] = kinect.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_RIGHT_KNEE,skeletonPoints[Skeleton.RIGHT_KNEE]);
			confidenceSkeletonPoints[Skeleton.RIGHT_FOOT] = kinect.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_RIGHT_FOOT,skeletonPoints[Skeleton.RIGHT_FOOT]);
		}
	}
	
	private void updateMirroredJointPositions () {
		if (mirrorPlaneCalculated) {
			switch (mirrorTherapy) {
				case MIRROR_THERAPY_LEFT:
					// mirror left elbow to right elbow
					float lDistanceToMP = PVector.dot(skeletonPoints[Skeleton.LEFT_ELBOW],n0MP) - dMP;
					skeletonPoints[Skeleton.RIGHT_ELBOW].set(PVector.add(skeletonPoints[Skeleton.LEFT_ELBOW],PVector.mult(n0MP,-2*lDistanceToMP)));
					confidenceSkeletonPoints[Skeleton.RIGHT_ELBOW] = confidenceSkeletonPoints[Skeleton.LEFT_ELBOW];
					// mirror left hand to right hand
					lDistanceToMP = PVector.dot(skeletonPoints[Skeleton.LEFT_HAND],n0MP) - dMP;
					skeletonPoints[Skeleton.RIGHT_HAND].set(PVector.add(skeletonPoints[Skeleton.LEFT_HAND],PVector.mult(n0MP,-2*lDistanceToMP)));
					confidenceSkeletonPoints[Skeleton.RIGHT_HAND] = confidenceSkeletonPoints[Skeleton.LEFT_HAND];
					break;
				case MIRROR_THERAPY_RIGHT:
					// mirror right elbow to left elbow
					float rDistanceToMP = PVector.dot(skeletonPoints[Skeleton.RIGHT_ELBOW],n0MP) - dMP;
					skeletonPoints[Skeleton.LEFT_ELBOW].set(PVector.add(skeletonPoints[Skeleton.RIGHT_ELBOW],PVector.mult(n0MP,-2*rDistanceToMP)));
					confidenceSkeletonPoints[Skeleton.LEFT_ELBOW] = confidenceSkeletonPoints[Skeleton.RIGHT_ELBOW];
					// mirror right hand to left hand
					rDistanceToMP = PVector.dot(skeletonPoints[Skeleton.RIGHT_HAND],n0MP) - dMP;
					skeletonPoints[Skeleton.LEFT_HAND].set(PVector.add(skeletonPoints[Skeleton.RIGHT_HAND],PVector.mult(n0MP,-2*rDistanceToMP)));
					confidenceSkeletonPoints[Skeleton.LEFT_HAND] = confidenceSkeletonPoints[Skeleton.RIGHT_HAND];
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
		skeletonPointsLocal[Skeleton.HEAD] = getLocalVector(skeletonPoints[Skeleton.HEAD]);
		skeletonPointsLocal[Skeleton.NECK] = getLocalVector(skeletonPoints[Skeleton.NECK]);
		skeletonPointsLocal[Skeleton.TORSO] = getLocalVector(skeletonPoints[Skeleton.TORSO]);
		skeletonPointsLocal[Skeleton.LEFT_SHOULDER] = getLocalVector(skeletonPoints[Skeleton.LEFT_SHOULDER]);
		skeletonPointsLocal[Skeleton.RIGHT_SHOULDER] = getLocalVector(skeletonPoints[Skeleton.RIGHT_SHOULDER]);
		skeletonPointsLocal[Skeleton.LEFT_ELBOW] = getLocalVector(skeletonPoints[Skeleton.LEFT_ELBOW]);
		skeletonPointsLocal[Skeleton.RIGHT_ELBOW] = getLocalVector(skeletonPoints[Skeleton.RIGHT_ELBOW]);
		skeletonPointsLocal[Skeleton.LEFT_HAND] = getLocalVector(skeletonPoints[Skeleton.LEFT_HAND]);
		skeletonPointsLocal[Skeleton.RIGHT_HAND] = getLocalVector(skeletonPoints[Skeleton.RIGHT_HAND]);
		if (fullBodyTracking) {
			skeletonPointsLocal[Skeleton.LEFT_HIP] = getLocalVector(skeletonPoints[Skeleton.LEFT_HIP]);
			skeletonPointsLocal[Skeleton.LEFT_KNEE] = getLocalVector(skeletonPoints[Skeleton.LEFT_KNEE]);
			skeletonPointsLocal[Skeleton.LEFT_FOOT] = getLocalVector(skeletonPoints[Skeleton.LEFT_FOOT]);
			skeletonPointsLocal[Skeleton.RIGHT_HIP] = getLocalVector(skeletonPoints[Skeleton.RIGHT_HIP]);
			skeletonPointsLocal[Skeleton.RIGHT_KNEE] = getLocalVector(skeletonPoints[Skeleton.RIGHT_KNEE]);
			skeletonPointsLocal[Skeleton.RIGHT_FOOT] = getLocalVector(skeletonPoints[Skeleton.RIGHT_FOOT]);
		}
	}
	
}
	