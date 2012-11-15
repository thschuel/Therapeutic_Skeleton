package therapeuticskeleton;

import processing.core.*;

public class SkeletonMath {

	// the skeleton all math is performed upon 
	private Skeleton skeleton = null;
	
	// local coordination system of skeleton
	private PVector origin;
	private PVector orientationX, orientationY, orientationZ;
	private PMatrix3D transformCoordSys;
	private PMatrix3D transformCoordSysInv;

	// calculation of mirror plane
	private PVector	rMP = new PVector(); // MirrorPlane in HNF: r*n0-d=0
	private PVector	n0MP = new PVector();
	private float dMP = 0.0f;
	
	public SkeletonMath (Skeleton _skeleton) {
		skeleton = _skeleton;
	}

	// GETTERS
	public PVector getOrigin() {
		return origin;
	}
	public PVector getOrientationX() {
		return orientationX;
	}
	public PVector getOrientationY() {
		return orientationY;
	}
	public PVector getOrientationZ() {
		return orientationZ;
	}
	public PVector getRMP() {
		return rMP;
	}
	public PVector getN0MP() {
		return n0MP;
	}
	public float getDMP() {
		return dMP;
	}
	
	// evaluate local coord sys
	// origin: neck
	// orientation: 
	// +x==right_shoulder-left_shoulder, 
	// -y==orthogonal on +x and pointing to torso
	// +z==cross product of x and y
	public void calculateLocalCoordSys () {
		origin = skeleton.getJoint(Skeleton.NECK);
		// *** calculating local coordSys
		// +x-axis==right_shoulder-left_shoulder, 
		orientationX = PVector.sub(skeleton.getJoint(Skeleton.RIGHT_SHOULDER),skeleton.getJoint(Skeleton.LEFT_SHOULDER));
		// +y==orthogonal to +x-axis, pointing from torso to x-axis. 
		// task: find point on orientationX
		// - the plane that contains torso and has orientationX as normal vector is defined as: 
		// - x1*orientationX.x+x2*orientationX.y+x3*orientationX.z=torso(dot)orientationX
		// - straight line defined by orientationX: [x] = left_shoulder+lambda*orientationX
		// - find lambda of crosspoint of that line with the plane: insert straight line as X into plane
		// - use lambda in straight line equation to get crosspoint
		// - +y is crosspoint-torso
		float lambda = skeleton.getJoint(Skeleton.TORSO).dot(orientationX);
		lambda -= orientationX.dot(skeleton.getJoint(Skeleton.LEFT_SHOULDER)); 
		lambda /= orientationX.dot(orientationX);
		PVector crossPoint = PVector.add(skeleton.getJoint(Skeleton.LEFT_SHOULDER),PVector.mult(orientationX,lambda));
		orientationY = PVector.sub(crossPoint,skeleton.getJoint(Skeleton.TORSO));
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
	}
	// transform joint coordinates to local coordsys. 
	public PVector getJointLCS (PVector globalVector) {
			PVector localVector = new PVector();
			transformCoordSysInv.mult(globalVector,localVector);
			return localVector;
	}
	
	// MIRROR THERAPY CAPABILITY
	// calculate mirror plane
	public void calculateMirrorPlane () {
		// calculate body plane of Shoulder and Torso points in HNF
		// HNF: r*n0-d = 0
		PVector r;
		PVector n0;
		// r is position vector of any point in the plane
		r = skeleton.getJoint(Skeleton.TORSO);
		// n0 is cross product of two vectors in the plane
		PVector temp1 = PVector.sub(skeleton.getJoint(Skeleton.LEFT_SHOULDER),r);
		PVector temp2 = PVector.sub(skeleton.getJoint(Skeleton.RIGHT_SHOULDER),r);
		n0 = temp1.cross(temp2);
		n0.normalize();
		// mirrorPlane is orthogonal to body plane and contains the line between torso and neck
		// calculate mirrorPlane in HNF: r*n0-d = 0
		rMP = r; // r is always set to position of the torso
		n0MP = n0.cross(PVector.sub(skeleton.getJoint(Skeleton.NECK),skeleton.getJoint(Skeleton.TORSO)));
		n0MP.normalize();
		dMP = PVector.dot(rMP,n0MP);
	}
	
	// mirror joint
	public PVector mirrorJointVector (PVector mirrorJoint) {
		float distanceToMP = PVector.dot(mirrorJoint,n0MP) - dMP;
		return PVector.add(mirrorJoint,PVector.mult(n0MP,-2*distanceToMP));
	}
	
	// mirror joint orientation
	public PMatrix3D mirrorOrientationMatrix (PMatrix3D mirrorMatrix) {
		PVector x = new PVector(mirrorMatrix.m00,mirrorMatrix.m10,mirrorMatrix.m20);
		PVector y = new PVector(mirrorMatrix.m01,mirrorMatrix.m11,mirrorMatrix.m21);
		PVector z = new PVector(mirrorMatrix.m02,mirrorMatrix.m12,mirrorMatrix.m22);
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
		return new PMatrix3D(-x.x,y.x,z.x,mirrorMatrix.m03,-x.y,y.y,z.y,mirrorMatrix.m13,-x.z,y.z,z.z,mirrorMatrix.m23,mirrorMatrix.m30,mirrorMatrix.m31,mirrorMatrix.m32,mirrorMatrix.m33);		
	}
	
	public static boolean isValueBetween (float val, float lowerBound, float upperBound) {
			return (val >= lowerBound && val <= upperBound);
	}
}
