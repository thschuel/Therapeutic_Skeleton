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

	// body planes in Hesse Normal Form, HNF: r*n0-d=0
	private BodyPlaneHNF sagittal = new BodyPlaneHNF(); // sagittal plane is mirror plane
	private BodyPlaneHNF frontal = new BodyPlaneHNF();
	private BodyPlaneHNF transversal = new BodyPlaneHNF();

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
	public BodyPlaneHNF getSagittalPlane() {
		return sagittal;
	}
	public BodyPlaneHNF getFrontalPlane() {
		return frontal;
	}
	public BodyPlaneHNF getTransversalPlane() {
		return transversal;
	}
	
	// evaluate local coord sys
	// origin: torso
	// orientation: 
	// +x-axis==left_shoulder -> right_shoulder
	// -y-axis==orthogonal on +x and pointing to torso
	// +z-axis==cross product of x and y
	public void calculateLocalCoordSys () {
		origin = skeleton.getJoint(Skeleton.TORSO);
		// *** calculating local coordSys
		// +x-axis==left_shoulder -> right_shoulder, 
		orientationX = PVector.sub(skeleton.getJoint(Skeleton.RIGHT_SHOULDER),skeleton.getJoint(Skeleton.LEFT_SHOULDER));
		// +y==orthogonal to +x-axis, pointing from torso to x-axis. 
		// task: find point on orientationX
		// - the plane that contains torso and has orientationX as normal vector is defined as: 
		// - x1*orientationX.x+x2*orientationX.y+x3*orientationX.z=torso(dot)orientationX
		// - straight line defined by orientationX: [x] = left_shoulder+lambda*orientationX
		// - find lambda of crosspoint of that line with the plane: insert straight line as X into plane
		// - use lambda in straight line equation to get crosspoint
		// - +y is crosspoint-torso
		float lambda = origin.dot(orientationX);
		lambda -= orientationX.dot(skeleton.getJoint(Skeleton.LEFT_SHOULDER)); 
		lambda /= orientationX.dot(orientationX);
		PVector crossPoint = PVector.add(skeleton.getJoint(Skeleton.LEFT_SHOULDER),PVector.mult(orientationX,lambda));
		orientationY = PVector.sub(crossPoint,origin);
		// =z-axis is cross product of y and x axis
		orientationZ = orientationX.cross(orientationY);
		
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
	// calculate body planes in HNF, Sagittal body plane is mirror plane
	// planes are given through local coordinate system. n0 of planes is the corresponding origin vector, r is always torso
	public void calculateBodyPlanes () {
		
		frontal.r = sagittal.r = transversal.r = origin;
		frontal.n0 = orientationZ;
		sagittal.n0 = orientationX;
		transversal.n0 = orientationY;
		frontal.d = PVector.dot(frontal.r,frontal.n0);
		sagittal.d = PVector.dot(sagittal.r,sagittal.n0);
		transversal.d = PVector.dot(transversal.r,transversal.n0);
		/* OLD CALCULATION. IS REDUNDANT SINCE LCS HOLDS THE SAME INFORMATION
		// calculate frontal body plane defined by Shoulder and Torso points in HNF
		// HNF: r*n0-d = 0
		// r is position vector of any point in the plane
		frontal.r = skeleton.getJoint(Skeleton.TORSO);
		// n0 is cross product of two vectors in the plane
		PVector temp1 = PVector.sub(skeleton.getJoint(Skeleton.LEFT_SHOULDER),frontal.r);
		PVector temp2 = PVector.sub(skeleton.getJoint(Skeleton.RIGHT_SHOULDER),frontal.r);
		frontal.n0 = temp2.cross(temp1);
		frontal.n0.normalize();
		frontal.d = PVector.dot(frontal.r,frontal.n0);
		
		// sagittal plane is orthogonal to frontal plane and contains the line between torso and neck
		// calculate mirrorPlane in HNF: r*n0-d = 0
		sagittal.r = skeleton.getJoint(Skeleton.TORSO); // r is always set to position of the torso
		sagittal.n0 = frontal.n0.cross(PVector.sub(skeleton.getJoint(Skeleton.NECK),skeleton.getJoint(Skeleton.TORSO)));
		sagittal.n0.normalize();
		sagittal.d = PVector.dot(sagittal.r,sagittal.n0);
		
		// transversal plane is orthogonal to sagittal and frontal plane and contains the torso point
		transversal.r = skeleton.getJoint(Skeleton.TORSO);
		transversal.n0 = frontal.n0.cross(sagittal.n0);
		transversal.n0.normalize();
		transversal.d = PVector.dot(transversal.r,transversal.n0);
		*/
	}
	
	// mirror joint
	public PVector mirrorJointVector (PVector mirrorJoint) {
		float distanceToMP = PVector.dot(mirrorJoint,sagittal.n0) - sagittal.d;
		return PVector.add(mirrorJoint,PVector.mult(sagittal.n0,-2*distanceToMP));
	}
	
	// mirror joint orientation
	public PMatrix3D mirrorOrientationMatrix (PMatrix3D mirrorMatrix) {
		PVector x = new PVector(mirrorMatrix.m00,mirrorMatrix.m10,mirrorMatrix.m20);
		PVector y = new PVector(mirrorMatrix.m01,mirrorMatrix.m11,mirrorMatrix.m21);
		PVector z = new PVector(mirrorMatrix.m02,mirrorMatrix.m12,mirrorMatrix.m22);
		x.add(sagittal.r);
		y.add(sagittal.r);
		z.add(sagittal.r);
		float distanceToMP = PVector.dot(x,sagittal.n0) - sagittal.d;
		x.set(PVector.add(x,PVector.mult(sagittal.n0,-2*distanceToMP)));
		distanceToMP = PVector.dot(y,sagittal.n0) - sagittal.d;
		y.set(PVector.add(y,PVector.mult(sagittal.n0,-2*distanceToMP)));
		distanceToMP = PVector.dot(z,sagittal.n0) - sagittal.d;
		z.set(PVector.add(z,PVector.mult(sagittal.n0,-2*distanceToMP)));
		x.sub(sagittal.r);
		y.sub(sagittal.r);
		z.sub(sagittal.r);
		return new PMatrix3D(-x.x,y.x,z.x,mirrorMatrix.m03,-x.y,y.y,z.y,mirrorMatrix.m13,-x.z,y.z,z.z,mirrorMatrix.m23,mirrorMatrix.m30,mirrorMatrix.m31,mirrorMatrix.m32,mirrorMatrix.m33);		
	}
	
	public static boolean isValueBetween (float val, float lowerBound, float upperBound) {
			return (val >= lowerBound && val <= upperBound);
	}
}

class BodyPlaneHNF {
	// HNF: r*n0-d=0
	public PVector r = new PVector();
	public PVector n0 = new PVector();
	public float d = 0.0f;
}
