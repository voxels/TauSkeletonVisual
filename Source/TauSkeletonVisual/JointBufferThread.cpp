// Fill out your copyright notice in the Description page of Project Settings.


#include "JointBufferThread.h"
#include "NuitrackSkeletonJointBuffer.h"
#include "TauBuffer.h"
#include "UObject/UObjectGlobals.h"
#include "Math/Vector.h"
#include "Kismet/KismetMathLibrary.h"

// Classes below from circumcenter.cpp in MeshKit   https://bitbucket.org/fathomteam/meshkit.git
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <string>
#include <chrono>
#include <sstream>
#include <iostream>

using namespace std;
using namespace std::chrono;

//
//Let a, b, c, d, and m be vectors in R^3.  Let ax, ay, and az be the components
//of a, and likewise for b, c, and d.  Let |a| denote the Euclidean norm of a,
//and let a x b denote the cross product of a and b.  Then
//
//    |                                                                       |
//    | |d-a|^2 [(b-a)x(c-a)] + |c-a|^2 [(d-a)x(b-a)] + |b-a|^2 [(c-a)x(d-a)] |
//    |                                                                       |
//r = -------------------------------------------------------------------------,
//                              | bx-ax  by-ay  bz-az |
//                            2 | cx-ax  cy-ay  cz-az |
//                              | dx-ax  dy-ay  dz-az |
//
//and
//
//        |d-a|^2 [(b-a)x(c-a)] + |c-a|^2 [(d-a)x(b-a)] + |b-a|^2 [(c-a)x(d-a)]
//m = a + ---------------------------------------------------------------------.
//                                | bx-ax  by-ay  bz-az |
//                              2 | cx-ax  cy-ay  cz-az |
//                                | dx-ax  dy-ay  dz-az |
//
//Some notes on stability:
//
//- Note that the expression for r is purely a function of differences between
//  coordinates.  The advantage is that the relative error incurred in the
//  computation of r is also a function of the _differences_ between the
//  vertices, and is not influenced by the _absolute_ coordinates of the
//  vertices.  In most applications, vertices are usually nearer to each other
//  than to the origin, so this property is advantageous.  If someone gives you
//  a formula that doesn't have this property, be wary.
//
//  Similarly, the formula for m incurs roundoff error proportional to the
//  differences between vertices, but does not incur roundoff error proportional
//  to the absolute coordinates of the vertices until the final addition.

//- These expressions are unstable in only one case:  if the denominator is
//  close to zero.  This instability, which arises if the tetrahedron is nearly
//  degenerate, is unavoidable.  Depending on your application, you may want
//  to use exact arithmetic to compute the value of the determinant.
//  Fortunately, this determinant is the basis of the well-studied 3D orientation
//  test, and several fast algorithms for providing accurate approximations to
//  the determinant are available.  Some resources are available from the
//  "Numerical and algebraic computation" page of Nina Amenta's Directory of
//  Computational Geometry Software:

//  http://www.geom.umn.edu/software/cglist/alg.html

//  If you're using floating-point inputs (as opposed to integers), one
//  package that can estimate this determinant somewhat accurately is my own:

//  http://www.cs.cmu.edu/~quake/robust.html

//- If you want to be even more aggressive about stability, you might reorder
//  the vertices of the tetrahedron so that the vertex `a' (which is subtracted
//  from the other vertices) is, roughly speaking, the vertex at the center of
//  the minimum spanning tree of the tetrahedron's four vertices.  For more
//  information about this idea, see Steven Fortune, "Numerical Stability of
//  Algorithms for 2D Delaunay Triangulations," International Journal of
//  Computational Geometry & Applications 5(1-2):193-213, March-June 1995.

//For completeness, here are stable expressions for the circumradius and
//circumcenter of a triangle, in R^2 and in R^3.  Incidentally, the expressions
//given here for R^2 are better behaved in terms of relative error than the
//suggestions currently given in the Geometry Junkyard
//(http://www.ics.uci.edu/~eppstein/junkyard/triangulation.html).

//Triangle in R^2:
//
//     |b-a| |c-a| |b-c|            < Note: You only want to compute one sqrt, so
//r = ------------------,             use sqrt{ |b-a|^2 |c-a|^2 |b-c|^2 }
//      | bx-ax  by-ay |
//    2 | cx-ax  cy-ay |
//
//          | by-ay  |b-a|^2 |
//          | cy-ay  |c-a|^2 |
//mx = ax - ------------------,
//            | bx-ax  by-ay |
//          2 | cx-ax  cy-ay |
//
//          | bx-ax  |b-a|^2 |
//          | cx-ax  |c-a|^2 |
//my = ay + ------------------.
//            | bx-ax  by-ay |
//          2 | cx-ax  cy-ay |
//
//Triangle in R^3:
//
//    |                                                           |
//    | |c-a|^2 [(b-a)x(c-a)]x(b-a) + |b-a|^2 (c-a)x[(b-a)x(c-a)] |
//    |                                                           |
//r = -------------------------------------------------------------,
//                         2 | (b-a)x(c-a) |^2
//
//        |c-a|^2 [(b-a)x(c-a)]x(b-a) + |b-a|^2 (c-a)x[(b-a)x(c-a)]
//m = a + ---------------------------------------------------------.
//                           2 | (b-a)x(c-a) |^2
//
//Finally, here's some C code that computes the vector m-a (whose norm is r) in
//each of these three cases.  Notice the #ifdef statements, which allow you to
//choose whether or not to use my aforementioned package to approximate the
//determinant.  (No attempt is made here to reorder the vertices to improve
//stability.)

/*****************************************************************************/
/*                                                                           */
/*  tetcircumcenter()   Find the circumcenter of a tetrahedron.              */
/*                                                                           */
/*  The result is returned both in terms of xyz coordinates and xi-eta-zeta  */
/*  coordinates, relative to the tetrahedron's point `a' (that is, `a' is    */
/*  the origin of both coordinate systems).  Hence, the xyz coordinates      */
/*  returned are NOT absolute; one must add the coordinates of `a' to        */
/*  find the absolute coordinates of the circumcircle.  However, this means  */
/*  that the result is frequently more accurate than would be possible if    */
/*  absolute coordinates were returned, due to limited floating-point        */
/*  precision.  In general, the circumradius can be computed much more       */
/*  accurately.                                                              */
/*                                                                           */
/*  The xi-eta-zeta coordinate system is defined in terms of the             */
/*  tetrahedron.  Point `a' is the origin of the coordinate system.          */
/*  The edge `ab' extends one unit along the xi axis.  The edge `ac'         */
/*  extends one unit along the eta axis.  The edge `ad' extends one unit     */
/*  along the zeta axis.  These coordinate values are useful for linear      */
/*  interpolation.                                                           */
/*                                                                           */
/*  If `xi' is NULL on input, the xi-eta-zeta coordinates will not be        */
/*  computed.                                                                */
/*                                                                           */
/*****************************************************************************/

/*****************************************************************************/

void tetcircumcenter(double a[3], double b[3], double c[3], double d[3],
	double circumcenter[3], double* xi, double* eta, double* zeta)
{
	double xba, yba, zba, xca, yca, zca, xda, yda, zda;
	double balength, calength, dalength;
	double xcrosscd, ycrosscd, zcrosscd;
	double xcrossdb, ycrossdb, zcrossdb;
	double xcrossbc, ycrossbc, zcrossbc;
	double denominator;
	double xcirca, ycirca, zcirca;

	/* Use coordinates relative to point `a' of the tetrahedron. */
	xba = b[0] - a[0];
	yba = b[1] - a[1];
	zba = b[2] - a[2];
	xca = c[0] - a[0];
	yca = c[1] - a[1];
	zca = c[2] - a[2];
	xda = d[0] - a[0];
	yda = d[1] - a[1];
	zda = d[2] - a[2];
	/* Squares of lengths of the edges incident to `a'. */
	balength = xba * xba + yba * yba + zba * zba;
	calength = xca * xca + yca * yca + zca * zca;
	dalength = xda * xda + yda * yda + zda * zda;
	/* Cross products of these edges. */
	xcrosscd = yca * zda - yda * zca;
	ycrosscd = zca * xda - zda * xca;
	zcrosscd = xca * yda - xda * yca;
	xcrossdb = yda * zba - yba * zda;
	ycrossdb = zda * xba - zba * xda;
	zcrossdb = xda * yba - xba * yda;
	xcrossbc = yba * zca - yca * zba;
	ycrossbc = zba * xca - zca * xba;
	zcrossbc = xba * yca - xca * yba;

	/* Calculate the denominator of the formulae. */
#ifdef EXACT
  /* Use orient3d() from http://www.cs.cmu.edu/~quake/robust.html     */
  /*   to ensure a correctly signed (and reasonably accurate) result, */
  /*   avoiding any possibility of division by zero.                  */
	denominator = 0.5 / orient3d(b, c, d, a);
#else
  /* Take your chances with floating-point roundoff. */
	printf(" Warning: IEEE floating points used: Define -DEXACT in makefile \n");
	denominator = 0.5 / (xba * xcrosscd + yba * ycrosscd + zba * zcrosscd);
#endif

	/* Calculate offset (from `a') of circumcenter. */
	xcirca = (balength * xcrosscd + calength * xcrossdb + dalength * xcrossbc) *
		denominator;
	ycirca = (balength * ycrosscd + calength * ycrossdb + dalength * ycrossbc) *
		denominator;
	zcirca = (balength * zcrosscd + calength * zcrossdb + dalength * zcrossbc) *
		denominator;
	circumcenter[0] = xcirca;
	circumcenter[1] = ycirca;
	circumcenter[2] = zcirca;

	if (xi != (double*)NULL) {
		/* To interpolate a linear function at the circumcenter, define a    */
		/*   coordinate system with a xi-axis directed from `a' to `b',      */
		/*   an eta-axis directed from `a' to `c', and a zeta-axis directed  */
		/*   from `a' to `d'.  The values for xi, eta, and zeta are computed */
		/*   by Cramer's Rule for solving systems of linear equations.       */
		*xi = (xcirca * xcrosscd + ycirca * ycrosscd + zcirca * zcrosscd) *
			(2.0 * denominator);
		*eta = (xcirca * xcrossdb + ycirca * ycrossdb + zcirca * zcrossdb) *
			(2.0 * denominator);
		*zeta = (xcirca * xcrossbc + ycirca * ycrossbc + zcirca * zcrossbc) *
			(2.0 * denominator);
	}
}

/*****************************************************************************/
/*****************************************************************************/
/*                                                                           */
/*  tricircumcenter()   Find the circumcenter of a triangle.                 */
/*                                                                           */
/*  The result is returned both in terms of x-y coordinates and xi-eta       */
/*  coordinates, relative to the triangle's point `a' (that is, `a' is       */
/*  the origin of both coordinate systems).  Hence, the x-y coordinates      */
/*  returned are NOT absolute; one must add the coordinates of `a' to        */
/*  find the absolute coordinates of the circumcircle.  However, this means  */
/*  that the result is frequently more accurate than would be possible if    */
/*  absolute coordinates were returned, due to limited floating-point        */
/*  precision.  In general, the circumradius can be computed much more       */
/*  accurately.                                                              */
/*                                                                           */
/*  The xi-eta coordinate system is defined in terms of the triangle.        */
/*  Point `a' is the origin of the coordinate system.  The edge `ab' extends */
/*  one unit along the xi axis.  The edge `ac' extends one unit along the    */
/*  eta axis.  These coordinate values are useful for linear interpolation.  */
/*                                                                           */
/*  If `xi' is NULL on input, the xi-eta coordinates will not be computed.   */
/*                                                                           */
/*****************************************************************************/


/*****************************************************************************/
void tricircumcenter(double a[2], double b[2], double c[2], double circumcenter[2],
	double* xi, double* eta)
{
	double xba, yba, xca, yca;
	double balength, calength;
	double denominator;
	double xcirca, ycirca;

	/* Use coordinates relative to point `a' of the triangle. */
	xba = b[0] - a[0];
	yba = b[1] - a[1];
	xca = c[0] - a[0];
	yca = c[1] - a[1];
	/* Squares of lengths of the edges incident to `a'. */
	balength = xba * xba + yba * yba;
	calength = xca * xca + yca * yca;

	/* Calculate the denominator of the formulae. */
#ifdef EXACT
  /* Use orient2d() from http://www.cs.cmu.edu/~quake/robust.html     */
  /*   to ensure a correctly signed (and reasonably accurate) result, */
  /*   avoiding any possibility of division by zero.                  */
	denominator = 0.5 / orient2d(b, c, a);
#else
  /* Take your chances with floating-point roundoff. */
	denominator = 0.5 / (xba * yca - yba * xca);
#endif

	/* Calculate offset (from `a') of circumcenter. */
	xcirca = (yca * balength - yba * calength) * denominator;
	ycirca = (xba * calength - xca * balength) * denominator;
	circumcenter[0] = xcirca;
	circumcenter[1] = ycirca;

	if (xi != (double*)NULL) {
		/* To interpolate a linear function at the circumcenter, define a     */
		/*   coordinate system with a xi-axis directed from `a' to `b' and    */
		/*   an eta-axis directed from `a' to `c'.  The values for xi and eta */
		/*   are computed by Cramer's Rule for solving systems of linear      */
		/*   equations.                                                       */
		*xi = (xcirca * yca - ycirca * xca) * (2.0 * denominator);
		*eta = (ycirca * xba - xcirca * yba) * (2.0 * denominator);
	}
}

/****************************************************************************/

/*****************************************************************************/
/*                                                                           */
/*  tricircumcenter3d()   Find the circumcenter of a triangle in 3D.         */
/*                                                                           */
/*  The result is returned both in terms of xyz coordinates and xi-eta       */
/*  coordinates, relative to the triangle's point `a' (that is, `a' is       */
/*  the origin of both coordinate systems).  Hence, the xyz coordinates      */
/*  returned are NOT absolute; one must add the coordinates of `a' to        */
/*  find the absolute coordinates of the circumcircle.  However, this means  */
/*  that the result is frequently more accurate than would be possible if    */
/*  absolute coordinates were returned, due to limited floating-point        */
/*  precision.  In general, the circumradius can be computed much more       */
/*  accurately.                                                              */
/*                                                                           */
/*  The xi-eta coordinate system is defined in terms of the triangle.        */
/*  Point `a' is the origin of the coordinate system.  The edge `ab' extends */
/*  one unit along the xi axis.  The edge `ac' extends one unit along the    */
/*  eta axis.  These coordinate values are useful for linear interpolation.  */
/*                                                                           */
/*  If `xi' is NULL on input, the xi-eta coordinates will not be computed.   */
/*                                                                           */
/*****************************************************************************/
/*****************************************************************************/
void tricircumcenter3d(double a[3], double b[3], double c[3], double circumcenter[3],
	double* xi, double* eta)
{
	double xba, yba, zba, xca, yca, zca;
	double balength, calength;
	double xcrossbc, ycrossbc, zcrossbc;
	double denominator;
	double xcirca, ycirca, zcirca;

	/* Use coordinates relative to point `a' of the triangle. */
	xba = b[0] - a[0];
	yba = b[1] - a[1];
	zba = b[2] - a[2];
	xca = c[0] - a[0];
	yca = c[1] - a[1];
	zca = c[2] - a[2];
	/* Squares of lengths of the edges incident to `a'. */
	balength = xba * xba + yba * yba + zba * zba;
	calength = xca * xca + yca * yca + zca * zca;

	/* Cross product of these edges. */
#ifdef EXACT
  /* Use orient2d() from http://www.cs.cmu.edu/~quake/robust.html     */
  /*   to ensure a correctly signed (and reasonably accurate) result, */
  /*   avoiding any possibility of division by zero.                  */

	A[0] = b[1]; A[1] = b[2];
	B[0] = c[1]; B[1] = c[2];
	C[0] = a[1]; C[1] = a[2];
	xcrossbc = orient2d(A, B, C);

	A[0] = c[0]; A[1] = c[2];
	B[0] = b[0]; B[1] = b[2];
	C[0] = a[0]; C[1] = a[2];
	ycrossbc = orient2d(A, B, C);

	A[0] = b[0]; A[1] = b[1];
	B[0] = c[0]; B[1] = c[1];
	C[0] = a[0]; C[1] = a[1];
	zcrossbc = orient2d(A, B, C);

	/*
	xcrossbc = orient2d(b[1], b[2], c[1], c[2], a[1], a[2]);
	ycrossbc = orient2d(b[2], b[0], c[2], c[0], a[2], a[0]);
	zcrossbc = orient2d(b[0], b[1], c[0], c[1], a[0], a[1]);
	*/
#else
	printf(" Warning: IEEE floating points used: Define -DEXACT in makefile \n");
	/* Take your chances with floating-point roundoff. */
	xcrossbc = yba * zca - yca * zba;
	ycrossbc = zba * xca - zca * xba;
	zcrossbc = xba * yca - xca * yba;
#endif

	/* Calculate the denominator of the formulae. */
	denominator = 0.5 / (xcrossbc * xcrossbc + ycrossbc * ycrossbc +
		zcrossbc * zcrossbc);

	/* Calculate offset (from `a') of circumcenter. */
	xcirca = ((balength * yca - calength * yba) * zcrossbc -
		(balength * zca - calength * zba) * ycrossbc) * denominator;
	ycirca = ((balength * zca - calength * zba) * xcrossbc -
		(balength * xca - calength * xba) * zcrossbc) * denominator;
	zcirca = ((balength * xca - calength * xba) * ycrossbc -
		(balength * yca - calength * yba) * xcrossbc) * denominator;
	circumcenter[0] = xcirca;
	circumcenter[1] = ycirca;
	circumcenter[2] = zcirca;

	if (xi != (double*)NULL) {
		/* To interpolate a linear function at the circumcenter, define a     */
		/*   coordinate system with a xi-axis directed from `a' to `b' and    */
		/*   an eta-axis directed from `a' to `c'.  The values for xi and eta */
		/*   are computed by Cramer's Rule for solving systems of linear      */
		/*   equations.                                                       */

		/* There are three ways to do this calculation - using xcrossbc, */
		/*   ycrossbc, or zcrossbc.  Choose whichever has the largest    */
		/*   magnitude, to improve stability and avoid division by zero. */
		if (((xcrossbc >= ycrossbc) ^ (-xcrossbc > ycrossbc)) &&
			((xcrossbc >= zcrossbc) ^ (-xcrossbc > zcrossbc))) {
			*xi = (ycirca * zca - zcirca * yca) / xcrossbc;
			*eta = (zcirca * yba - ycirca * zba) / xcrossbc;
		}
		else if ((ycrossbc >= zcrossbc) ^ (-ycrossbc > zcrossbc)) {
			*xi = (zcirca * xca - xcirca * zca) / ycrossbc;
			*eta = (xcirca * zba - zcirca * xba) / ycrossbc;
		}
		else {
			*xi = (xcirca * yca - ycirca * xca) / zcrossbc;
			*eta = (ycirca * xba - xcirca * yba) / zcrossbc;
		}
	}
}
/****************************************************************************/
void TriCircumCenter2D(double* a, double* b, double* c, double* result,
	double* param)
{
	tricircumcenter(a, b, c, result, &param[0], &param[1]);

	result[0] += a[0];
	result[1] += a[1];
}
/****************************************************************************/
void TriCircumCenter3D(double* a, double* b, double* c, double* result,
	double* param)
{
	tricircumcenter3d(a, b, c, result, &param[0], &param[1]);
	result[0] += a[0];
	result[1] += a[1];
	result[2] += a[2];
}

/****************************************************************************/
void TriCircumCenter3D(double* a, double* b, double* c, double* result)
{
	double xi, eta;
	tricircumcenter3d(a, b, c, result, &xi, &eta);
	result[0] += a[0];
	result[1] += a[1];
	result[2] += a[2];
}

FJointBufferThread::FJointBufferThread(TArray<FName>_BoneNames, TArray<FVector>_Locations, TArray<FRotator>_Rotations, TArray<float>_Confidences, std::vector<UTauBuffer*>_PreviousTriangleTauBuffers, UNuitrackSkeletonJointBuffer* _JointBuffer)
{
	if (_JointBuffer) {
		SocketBoneNames = _BoneNames;
		SocketLocations = _Locations;
		SocketRotations = _Rotations;
		SocketConfidences = _Confidences;
		JointBuffer = _JointBuffer;
		TriangleTauBuffers = _PreviousTriangleTauBuffers;
	}
}

FJointBufferThread::~FJointBufferThread()
{
	TriangleTauBuffers.clear();
}


bool FJointBufferThread::Init()
{
	bStopThread = false;
	bProcessComplete = false;
	return true;
}

uint32 FJointBufferThread::Run()
{


	while (!bStopThread && !bProcessComplete) {
		ProcessSocketRawData();
		JointBuffer->TriangleIndexesQueue.Enqueue(TriangleIndexes);
		JointBuffer->TrianglePositionsQueue.Enqueue(TrianglePositions);
		JointBuffer->TriangleIndexBoneNamesQueue.Enqueue(TriangleIndexBoneNames);
		JointBuffer->TriangleRotationsQueue.Enqueue(TriangleRotations);
		JointBuffer->TriangleCentroidsQueue.Enqueue(TriangleCentroids);
		JointBuffer->TriangleCircumcentersQueue.Enqueue(TriangleCircumcenters);
		JointBuffer->EulerLinesQueue.Enqueue(EulerLines);
		JointBuffer->TriangleTauBuffersQueue.Enqueue(TriangleTauBuffers);
		bProcessComplete = true;
	}

	return 0;
}

void FJointBufferThread::Stop()
{

}

void FJointBufferThread::ProcessSocketRawData()
{
	if (SocketBoneNames.Num() < 1) {
		return;
	}

	UpdateTriangles(SocketBoneNames, SocketLocations, SocketRotations);
	UpdateDebugLines();
	UpdateEulerLines();
	UpdateTracking();
}



/*
[0] = joints[JointType::JOINT_HEAD]
[1] = joints[JointType::JOINT_NECK]
[2] = joints[JointType::JOINT_TORSO]
[3] joints[JointType::JOINT_WAIST]

[4] joints[JointType::JOINT_LEFT_SHOULDER]
[5] joints[JointType::JOINT_LEFT_ELBOW]
[6] joints[JointType::JOINT_LEFT_WRIST]
[7] joints[JointType::JOINT_LEFT_HAND]

[8] joints[JointType::JOINT_RIGHT_SHOULDER]
[9] joints[JointType::JOINT_RIGHT_ELBOW]
[10] joints[JointType::JOINT_RIGHT_WRIST]
[11] joints[JointType::JOINT_RIGHT_HAND]

[12] joints[JointType::JOINT_LEFT_HIP]
[13] joints[JointType::JOINT_LEFT_KNEE]
[14] joints[JointType::JOINT_LEFT_ANKLE]

[15] joints[JointType::JOINT_RIGHT_HIP]
[16] joints[JointType::JOINT_RIGHT_KNEE]
[17] joints[JointType::JOINT_RIGHT_ANKLE]
*/
void FJointBufferThread::UpdateTriangles(TArray<FName>BoneNames, TArray<FVector>Locations, TArray<FRotator>Rotations)
{
	TriangleIndexes = {
		1,
		12,
		15,
		1,
		14,
		17,
		3,
		4,
		8,
		3,
		7,
		11,
		3,
		12,
		15,
		1,
		8,
		2,
		1,
		8,
		3,
		1,
		9,
		10,
		1,
		9,
		17,
		3,
		8,
		1,
		3,
		8,
		10,
		3,
		15,
		17,
		3,
		9,
		10,
		15,
		8,
		12,
		15,
		8,
		9,
		15,
		8,
		10,
		15,
		1,
		8,
		15,
		9,
		10,
		15,
		16,
		13,
		15,
		16,
		17,
		16,
		8,
		9,
		16,
		8,
		13,
		16,
		15,
		12,
		16,
		17,
		14,
		17,
		8,
		4,
		17,
		16,
		13,
		// Left Side Head
1,		// Head
4,		// Left Shoulder
2,		// Neck

1,		// Head
4,		// Left Shoulder
3,		// Spine 1

1,		// Head
5,		// Left Arm
6,	// Left  Wrist

1,		// Head
5,		// Left Arm
14,	// Left Foot


//Left Side Chest
3,		// Spine 1
4,		// Left Shoulder
5,		// Left Arm

3,		// Spine 1
4,		// Left Shoulder
6,		// Left Wrist

3,		// Spine 1
12,	// Left Leg Up
14,	// Left Foot

3,		// Spine 1
5,		// Left Arm
6,	// Left Hand

// Left Side Hip
12,	// Left Leg Up 
4,		// Left Shoulder
15,	// Right Leg Up

12,	// Left Leg Up 
4,		// Left Shoulder
5,		// Left Arm

12,	// Left Leg Up 
4,		// Left Shoulder
6,	// Left Wrist

12,	// Left Leg Up  
2,		// Neck
4,		// Left Shoulder

12,	// Left Leg Up 
5,		// Left Arm
6,	// Left Wrist

12,	// Left Leg Up 
13,	// Left Leg
16,	// Right Leg

12,	// Left Leg Up 
13,	// Left Leg
14,	// Left Foot 

// Left Side Leg
13,	// Left Leg
4,		// Left Shoulder
5,		// Left Arm

13,	// Left Leg
4,		// Left Shoulder 
16,	// Right Leg

13,	// Left Leg
12,	// Left Leg Up
15,	// Right Leg Up

13,	// Left Leg
14,	// Left Foot 
17,	// Right Foot

// Left Side Ankle
14,	// Left Foot
4,		// Left Shoulder
8,	// Right Shoulder

14,	// Left Foot
13,	// Left Leg
16,	// Right Leg

// Cross Center
1,		// Head
9,	// Right Elbow
14,	// Left Foot

1,		// Head
5,		// Left Elbow
17,	// Right Foot

1,		// Head
9,	// Right Elbow
6,	// Left Wrist

1,		// Head
5,		// Left Elbow
10,	// Right WRist

3,		// Spine 1
15,	// Right Hip
6,	// Left Wrist

3,		// Spine 1
12,	// Left Hip
10,	// Right Wrist

3,		// Spine 1
8,	// Right Shoulder
6,	// Left Wrist

3,		// Spine 1
4,		// Left Shoulder
10,	// Right Wrist

15,	// Right Up Leg 
4,		// Left Shoulder
9,	// Right Arm

12,	// Left Leg Up 
8,	// Right Shoulder
5,		// Left Arm

15,	// Right Up Leg 
4,		// Left Shoulder
10,	// Right Wrist


12,	// Left Leg Up 
8,	// Right Shoulder
6,	// Left Wrist

15,	// Right Up Leg 
5,		// Left Arm
10,	// Right Wrist

12,	// Left Leg Up 
9,	// Right Arm
6,	// Left Wrist

15,	// Right Up Leg 
13,	// Left Leg
10,	// Right Wrist

12,	// Left Leg Up 
16,	// Right Leg
6,	// Left Wrist

15,	// Right Up Leg 
14,	// Left Foot
16,	// Right Leg

12,	// Left Leg Up 
17,	// Right Root
13,	// Left Leg

16,	// Right Leg
4,	// Left Shoulder
9,	// Right Arm

13,	// Left Leg
8,	// Right Shoulder
5	// Left Arm
	};


	TriangleIndexBoneNames =
	{
		// Center Symmetrical	
		BoneNames[1],	// Head 
		BoneNames[12],	// Left Up Leg
		BoneNames[15],	// Right Up Leg

		BoneNames[1],	// Head
		BoneNames[14],	// Left Top Base
		BoneNames[17],	// Right Toe Base

		BoneNames[3],	// Spine 1
		BoneNames[4],	// Left Shoulder
		BoneNames[8],	// Right Shoulder

		BoneNames[3],	// Spine 1
		BoneNames[7],	// Left Hand
		BoneNames[11],	// Right Hand

		BoneNames[3],	// Spine 1
		BoneNames[12],	// Left Up Leg
		BoneNames[15],	// Right Up Leg

		// Right Side Head
		BoneNames[1],		// Head
		BoneNames[8],	// Right Shoulder 
		BoneNames[2],		// Neck

		BoneNames[1],		// Head
		BoneNames[8],	// Right Shoulder
		BoneNames[3],		// Spine 1

		BoneNames[1],		// Head
		BoneNames[9],	// Right Arm
		BoneNames[10],	// Right Wrist 

		BoneNames[1],		// Head
		BoneNames[9],	// Right Arm 
		BoneNames[17],	// Right Foot 


		//Right Side Chest
		BoneNames[3],		// Spine 1
		BoneNames[8],	// Right Shoulder 
		BoneNames[1],		// Neck

		BoneNames[3],		// Spine 1 
		BoneNames[8],	// Right Shoulder 
		BoneNames[10],	// Right Wrist

		BoneNames[3],		// Spine 1 
		BoneNames[15],	// Right Up Leg 
		BoneNames[17],	// Right Foot

		BoneNames[3],		// Spine 1 
		BoneNames[9],	// Right Arm
		BoneNames[10],	// Right Hand 

		// Right Side Hip
		BoneNames[15],	// Right Up Leg 
		BoneNames[8],	// Right Shoulder 
		BoneNames[12],	// Left Up Leg 

		BoneNames[15],	// Right Up Leg 
		BoneNames[8],	// Right Shoulder 
		BoneNames[9],	// Right Arm 

		BoneNames[15],	// Right Up Leg
		BoneNames[8],	// Right Shoulder 
		BoneNames[10],	// Right Wrist

		BoneNames[15],	// Right Up Leg
		BoneNames[1],		// Neck
		BoneNames[8],	// Right Shoulder

		BoneNames[15],	// Right Up Leg
		BoneNames[9],	// Right Arm
		BoneNames[10],	// Right Wrist

		BoneNames[15],	// Right Up Leg
		BoneNames[16],	// Right Leg 
		BoneNames[13],	// Left Knee

		BoneNames[15],	// Right Up Leg
		BoneNames[16],	// Right Leg 
		BoneNames[17],	// Right Foot

		// Right Side Knee
		BoneNames[16],	// Right Leg 
		BoneNames[7],	// Right Shoulder
		BoneNames[9],	// Right Elbow

		BoneNames[16],	// Right Leg 
		BoneNames[8],	// Right Shoulder
		BoneNames[13],	// Left Leg

		BoneNames[16],	// Right Leg 
		BoneNames[15],	// Right Hip 
		BoneNames[12],	// Left Hip 

		BoneNames[16],	// Right Leg 
		BoneNames[17],	// Right Foot
		BoneNames[14],	// Left Foot

		// Right Side Ankle
		BoneNames[17],	// Right Foot 
		BoneNames[8],	// Right Shoulder
		BoneNames[4],		// Left Shoulder

		BoneNames[17],	// Right Foot 
		BoneNames[16],	// Right Leg
		BoneNames[13],	// Left Leg


		// Left Side Head
		BoneNames[1],		// Head
		BoneNames[4],		// Left Shoulder
		BoneNames[1],		// Neck

		BoneNames[1],		// Head
		BoneNames[4],		// Left Shoulder
		BoneNames[3],		// Spine 1

		BoneNames[1],		// Head
		BoneNames[5],		// Left Arm
		BoneNames[6],	// Left  Wrist

		BoneNames[1],		// Head
		BoneNames[5],		// Left Arm
		BoneNames[14],	// Left Foot


		//Left Side Chest
		BoneNames[3],		// Spine 1
		BoneNames[4],		// Left Shoulder
		BoneNames[5],		// Left Arm

		BoneNames[3],		// Spine 1
		BoneNames[4],		// Left Shoulder
		BoneNames[6],		// Left Wrist

		BoneNames[3],		// Spine 1
		BoneNames[12],	// Left Leg Up
		BoneNames[14],	// Left Foot

		BoneNames[3],		// Spine 1
		BoneNames[5],		// Left Arm
		BoneNames[6],	// Left Hand

		// Left Side Hip
		BoneNames[12],	// Left Leg Up 
		BoneNames[4],		// Left Shoulder
		BoneNames[15],	// Right Leg Up

		BoneNames[12],	// Left Leg Up 
		BoneNames[4],		// Left Shoulder
		BoneNames[5],		// Left Arm

		BoneNames[12],	// Left Leg Up 
		BoneNames[4],		// Left Shoulder
		BoneNames[6],	// Left Wrist

		BoneNames[12],	// Left Leg Up  
		BoneNames[2],		// Neck
		BoneNames[4],		// Left Shoulder

		BoneNames[12],	// Left Leg Up 
		BoneNames[5],		// Left Arm
		BoneNames[6],	// Left Wrist

		BoneNames[12],	// Left Leg Up 
		BoneNames[13],	// Left Leg
		BoneNames[14],	// Right Leg

		BoneNames[12],	// Left Leg Up 
		BoneNames[12],	// Left Leg
		BoneNames[13],	// Left Foot 

		// Left Side Leg
		BoneNames[13],	// Left Leg
		BoneNames[4],		// Left Shoulder
		BoneNames[5],		// Left Arm

		BoneNames[13],	// Left Leg
		BoneNames[4],		// Left Shoulder 
		BoneNames[16],	// Right Leg

		BoneNames[13],	// Left Leg
		BoneNames[12],	// Left Leg Up
		BoneNames[15],	// Right Leg Up

		BoneNames[13],	// Left Leg
		BoneNames[14],	// Left Foot 
		BoneNames[17],	// Right Foot

		// Left Side Ankle
		BoneNames[14],	// Left Foot
		BoneNames[4],		// Left Shoulder
		BoneNames[8],	// Right Shoulder

		BoneNames[14],	// Left Foot
		BoneNames[13],	// Left Leg
		BoneNames[16],	// Right Leg

		// Cross Center
		BoneNames[1],		// Head
		BoneNames[9],	// Right Elbow
		BoneNames[14],	// Left Foot

		BoneNames[1],		// Head
		BoneNames[5],		// Left Elbow
		BoneNames[17],	// Right Foot

		BoneNames[1],		// Head
		BoneNames[9],	// Right Elbow
		BoneNames[6],	// Left Wrist

		BoneNames[1],		// Head
		BoneNames[5],		// Left Elbow
		BoneNames[10],	// Right WRist

		BoneNames[3],		// Spine 1
		BoneNames[15],	// Right Hip
		BoneNames[6],	// Left Wrist

		BoneNames[3],		// Spine 1
		BoneNames[12],	// Left Hip
		BoneNames[10],	// Right Wrist

		BoneNames[3],		// Spine 1
		BoneNames[8],	// Right Shoulder
		BoneNames[6],	// Left Wrist

		BoneNames[3],		// Spine 1
		BoneNames[4],		// Left Shoulder
		BoneNames[10],	// Right Wrist

		BoneNames[15],	// Right Up Leg 
		BoneNames[4],		// Left Shoulder
		BoneNames[9],	// Right Arm

		BoneNames[12],	// Left Leg Up 
		BoneNames[8],	// Right Shoulder
		BoneNames[5],		// Left Arm

		BoneNames[15],	// Right Up Leg 
		BoneNames[4],		// Left Shoulder
		BoneNames[10],	// Right Wrist


		BoneNames[12],	// Left Leg Up 
		BoneNames[8],	// Right Shoulder
		BoneNames[6],	// Left Wrist

		BoneNames[15],	// Right Up Leg 
		BoneNames[5],		// Left Arm
		BoneNames[10],	// Right Wrist

		BoneNames[12],	// Left Leg Up 
		BoneNames[9],	// Right Arm
		BoneNames[6],	// Left Wrist

		BoneNames[15],	// Right Up Leg 
		BoneNames[13],	// Left Leg
		BoneNames[10],	// Right Wrist

		BoneNames[12],	// Left Leg Up 
		BoneNames[16],	// Right Leg
		BoneNames[6],	// Left Wrist

		BoneNames[15],	// Right Up Leg 
		BoneNames[14],	// Left Foot
		BoneNames[16],	// Right Leg

		BoneNames[12],	// Left Leg Up 
		BoneNames[17],	// Right Root
		BoneNames[13],	// Left Leg

		BoneNames[16],	// Right Leg
		BoneNames[4],	// Left Shoulder
		BoneNames[9],	// Right Arm

		BoneNames[13],	// Left Leg
		BoneNames[8],	// Right Shoulder
		BoneNames[5]	// Left Arm
	};


	TrianglePositions =
	{
		// Center Symmetrical	
		Locations[1],	// Head 
		Locations[12],	// Left Up Leg
		Locations[15],	// Right Up Leg

		Locations[1],	// Head
		Locations[14],	// Left Top Base
		Locations[17],	// Right Toe Base

		Locations[3],	// Spine 1
		Locations[4],	// Left Shoulder
		Locations[8],	// Right Shoulder

		Locations[3],	// Spine 1
		Locations[7],	// Left Hand
		Locations[11],	// Right Hand

		Locations[3],	// Spine 1
		Locations[12],	// Left Up Leg
		Locations[15],	// Right Up Leg

		// Right Side Head
		Locations[1],		// Head
		Locations[8],	// Right Shoulder 
		Locations[2],		// Neck

		Locations[1],		// Head
		Locations[8],	// Right Shoulder
		Locations[3],		// Spine 1

		Locations[1],		// Head
		Locations[9],	// Right Arm
		Locations[10],	// Right Wrist 

		Locations[1],		// Head
		Locations[9],	// Right Arm 
		Locations[17],	// Right Foot 


		//Right Side Chest
		Locations[3],		// Spine 1
		Locations[8],	// Right Shoulder 
		Locations[1],		// Neck

		Locations[3],		// Spine 1 
		Locations[8],	// Right Shoulder 
		Locations[10],	// Right Wrist

		Locations[3],		// Spine 1 
		Locations[15],	// Right Up Leg 
		Locations[17],	// Right Foot

		Locations[3],		// Spine 1 
		Locations[9],	// Right Arm
		Locations[10],	// Right Hand 

		// Right Side Hip
		Locations[15],	// Right Up Leg 
		Locations[8],	// Right Shoulder 
		Locations[12],	// Left Up Leg 

		Locations[15],	// Right Up Leg 
		Locations[8],	// Right Shoulder 
		Locations[9],	// Right Arm 

		Locations[15],	// Right Up Leg
		Locations[8],	// Right Shoulder 
		Locations[10],	// Right Wrist

		Locations[15],	// Right Up Leg
		Locations[1],		// Neck
		Locations[8],	// Right Shoulder

		Locations[15],	// Right Up Leg
		Locations[9],	// Right Arm
		Locations[10],	// Right Wrist

		Locations[15],	// Right Up Leg
		Locations[16],	// Right Leg 
		Locations[13],	// Left Knee

		Locations[15],	// Right Up Leg
		Locations[16],	// Right Leg 
		Locations[17],	// Right Foot

		// Right Side Knee
		Locations[16],	// Right Leg 
		Locations[7],	// Right Shoulder
		Locations[9],	// Right Elbow

		Locations[16],	// Right Leg 
		Locations[8],	// Right Shoulder
		Locations[13],	// Left Leg

		Locations[16],	// Right Leg 
		Locations[15],	// Right Hip 
		Locations[12],	// Left Hip 

		Locations[16],	// Right Leg 
		Locations[17],	// Right Foot
		Locations[14],	// Left Foot

		// Right Side Ankle
		Locations[17],	// Right Foot 
		Locations[8],	// Right Shoulder
		Locations[4],		// Left Shoulder

		Locations[17],	// Right Foot 
		Locations[16],	// Right Leg
		Locations[13],	// Left Leg


		// Left Side Head
		Locations[1],		// Head
		Locations[4],		// Left Shoulder
		Locations[2],		// Neck

		Locations[1],		// Head
		Locations[4],		// Left Shoulder
		Locations[3],		// Spine 1

		Locations[1],		// Head
		Locations[5],		// Left Arm
		Locations[6],	// Left  Wrist

		Locations[1],		// Head
		Locations[5],		// Left Arm
		Locations[14],	// Left Foot


		//Left Side Chest
		Locations[3],		// Spine 1
		Locations[4],		// Left Shoulder
		Locations[5],		// Left Arm

		Locations[3],		// Spine 1
		Locations[4],		// Left Shoulder
		Locations[6],		// Left Wrist

		Locations[3],		// Spine 1
		Locations[12],	// Left Leg Up
		Locations[14],	// Left Foot

		Locations[3],		// Spine 1
		Locations[5],		// Left Arm
		Locations[6],	// Left Hand

		// Left Side Hip
		Locations[12],	// Left Leg Up 
		Locations[4],		// Left Shoulder
		Locations[15],	// Right Leg Up

		Locations[12],	// Left Leg Up 
		Locations[4],		// Left Shoulder
		Locations[5],		// Left Arm

		Locations[12],	// Left Leg Up 
		Locations[4],		// Left Shoulder
		Locations[6],	// Left Wrist

		Locations[12],	// Left Leg Up  
		Locations[2],		// Neck
		Locations[4],		// Left Shoulder

		Locations[12],	// Left Leg Up 
		Locations[5],		// Left Arm
		Locations[6],	// Left Wrist

		Locations[12],	// Left Leg Up 
		Locations[13],	// Left Leg
		Locations[14],	// Right Leg

		Locations[12],	// Left Leg Up 
		Locations[12],	// Left Leg
		Locations[13],	// Left Foot 

		// Left Side Leg
		Locations[13],	// Left Leg
		Locations[4],		// Left Shoulder
		Locations[5],		// Left Arm

		Locations[13],	// Left Leg
		Locations[4],		// Left Shoulder 
		Locations[16],	// Right Leg

		Locations[13],	// Left Leg
		Locations[12],	// Left Leg Up
		Locations[15],	// Right Leg Up

		Locations[13],	// Left Leg
		Locations[14],	// Left Foot 
		Locations[17],	// Right Foot

		// Left Side Ankle
		Locations[14],	// Left Foot
		Locations[4],		// Left Shoulder
		Locations[8],	// Right Shoulder

		Locations[14],	// Left Foot
		Locations[13],	// Left Leg
		Locations[16],	// Right Leg

		// Cross Center
		Locations[1],		// Head
		Locations[9],	// Right Elbow
		Locations[14],	// Left Foot

		Locations[1],		// Head
		Locations[5],		// Left Elbow
		Locations[17],	// Right Foot

		Locations[1],		// Head
		Locations[9],	// Right Elbow
		Locations[6],	// Left Wrist

		Locations[1],		// Head
		Locations[5],		// Left Elbow
		Locations[10],	// Right WRist

		Locations[3],		// Spine 1
		Locations[15],	// Right Hip
		Locations[6],	// Left Wrist

		Locations[3],		// Spine 1
		Locations[12],	// Left Hip
		Locations[10],	// Right Wrist

		Locations[3],		// Spine 1
		Locations[8],	// Right Shoulder
		Locations[6],	// Left Wrist

		Locations[3],		// Spine 1
		Locations[4],		// Left Shoulder
		Locations[10],	// Right Wrist

		Locations[15],	// Right Up Leg 
		Locations[4],		// Left Shoulder
		Locations[9],	// Right Arm

		Locations[12],	// Left Leg Up 
		Locations[8],	// Right Shoulder
		Locations[5],		// Left Arm

		Locations[15],	// Right Up Leg 
		Locations[4],		// Left Shoulder
		Locations[10],	// Right Wrist


		Locations[12],	// Left Leg Up 
		Locations[8],	// Right Shoulder
		Locations[6],	// Left Wrist

		Locations[15],	// Right Up Leg 
		Locations[5],		// Left Arm
		Locations[10],	// Right Wrist

		Locations[12],	// Left Leg Up 
		Locations[9],	// Right Arm
		Locations[6],	// Left Wrist

		Locations[15],	// Right Up Leg 
		Locations[13],	// Left Leg
		Locations[10],	// Right Wrist

		Locations[12],	// Left Leg Up 
		Locations[16],	// Right Leg
		Locations[6],	// Left Wrist

		Locations[15],	// Right Up Leg 
		Locations[14],	// Left Foot
		Locations[16],	// Right Leg

		Locations[12],	// Left Leg Up 
		Locations[17],	// Right Root
		Locations[13],	// Left Leg

		Locations[16],	// Right Leg
		Locations[4],	// Left Shoulder
		Locations[9],	// Right Arm

		Locations[13],	// Left Leg
		Locations[8],	// Right Shoulder
		Locations[5]	// Left Arm
	};

	TriangleRotations =
	{
		// Center Symmetrical	
		Rotations[1],	// Head 
		Rotations[12],	// Left Up Leg
		Rotations[15],	// Right Up Leg

		Rotations[1],	// Head
		Rotations[14],	// Left Top Base
		Rotations[17],	// Right Toe Base

		Rotations[3],	// Spine 1
		Rotations[4],	// Left Shoulder
		Rotations[8],	// Right Shoulder

		Rotations[3],	// Spine 1
		Rotations[7],	// Left Hand
		Rotations[11],	// Right Hand

		Rotations[3],	// Spine 1
		Rotations[12],	// Left Up Leg
		Rotations[15],	// Right Up Leg

		// Right Side Head
		Rotations[1],		// Head
		Rotations[8],	// Right Shoulder 
		Rotations[2],		// Neck

		Rotations[1],		// Head
		Rotations[8],	// Right Shoulder
		Rotations[3],		// Spine 1

		Rotations[1],		// Head
		Rotations[9],	// Right Arm
		Rotations[10],	// Right Wrist 

		Rotations[1],		// Head
		Rotations[9],	// Right Arm 
		Rotations[17],	// Right Foot 


		//Right Side Chest
		Rotations[3],		// Spine 1
		Rotations[8],	// Right Shoulder 
		Rotations[1],		// Neck

		Rotations[3],		// Spine 1 
		Rotations[8],	// Right Shoulder 
		Rotations[10],	// Right Wrist

		Rotations[3],		// Spine 1 
		Rotations[15],	// Right Up Leg 
		Rotations[17],	// Right Foot

		Rotations[3],		// Spine 1 
		Rotations[9],	// Right Arm
		Rotations[10],	// Right Hand 

		// Right Side Hip
		Rotations[15],	// Right Up Leg 
		Rotations[8],	// Right Shoulder 
		Rotations[12],	// Left Up Leg 

		Rotations[15],	// Right Up Leg 
		Rotations[8],	// Right Shoulder 
		Rotations[9],	// Right Arm 

		Rotations[15],	// Right Up Leg
		Rotations[8],	// Right Shoulder 
		Rotations[10],	// Right Wrist

		Rotations[15],	// Right Up Leg
		Rotations[1],		// Neck
		Rotations[8],	// Right Shoulder

		Rotations[15],	// Right Up Leg
		Rotations[9],	// Right Arm
		Rotations[10],	// Right Wrist

		Rotations[15],	// Right Up Leg
		Rotations[16],	// Right Leg 
		Rotations[13],	// Left Knee

		Rotations[15],	// Right Up Leg
		Rotations[16],	// Right Leg 
		Rotations[17],	// Right Foot

		// Right Side Knee
		Rotations[16],	// Right Leg 
		Rotations[7],	// Right Shoulder
		Rotations[9],	// Right Elbow

		Rotations[16],	// Right Leg 
		Rotations[8],	// Right Shoulder
		Rotations[13],	// Left Leg

		Rotations[16],	// Right Leg 
		Rotations[15],	// Right Hip 
		Rotations[12],	// Left Hip 

		Rotations[16],	// Right Leg 
		Rotations[17],	// Right Foot
		Rotations[14],	// Left Foot

		// Right Side Ankle
		Rotations[17],	// Right Foot 
		Rotations[8],	// Right Shoulder
		Rotations[4],		// Left Shoulder

		Rotations[17],	// Right Foot 
		Rotations[16],	// Right Leg
		Rotations[13],	// Left Leg


		// Left Side Head
		Rotations[1],		// Head
		Rotations[4],		// Left Shoulder
		Rotations[2],		// Neck

		Rotations[1],		// Head
		Rotations[4],		// Left Shoulder
		Rotations[3],		// Spine 1

		Rotations[1],		// Head
		Rotations[5],		// Left Arm
		Rotations[6],	// Left  Wrist

		Rotations[1],		// Head
		Rotations[5],		// Left Arm
		Rotations[14],	// Left Foot


		//Left Side Chest
		Rotations[3],		// Spine 1
		Rotations[4],		// Left Shoulder
		Rotations[5],		// Left Arm

		Rotations[3],		// Spine 1
		Rotations[4],		// Left Shoulder
		Rotations[6],		// Left Wrist

		Rotations[3],		// Spine 1
		Rotations[12],	// Left Leg Up
		Rotations[14],	// Left Foot

		Rotations[3],		// Spine 1
		Rotations[5],		// Left Arm
		Rotations[6],	// Left Hand

		// Left Side Hip
		Rotations[12],	// Left Leg Up 
		Rotations[4],		// Left Shoulder
		Rotations[15],	// Right Leg Up

		Rotations[12],	// Left Leg Up 
		Rotations[4],		// Left Shoulder
		Rotations[5],		// Left Arm

		Rotations[12],	// Left Leg Up 
		Rotations[4],		// Left Shoulder
		Rotations[6],	// Left Wrist

		Rotations[12],	// Left Leg Up  
		Rotations[2],		// Neck
		Rotations[4],		// Left Shoulder

		Rotations[12],	// Left Leg Up 
		Rotations[5],		// Left Arm
		Rotations[6],	// Left Wrist

		Rotations[12],	// Left Leg Up 
		Rotations[13],	// Left Leg
		Rotations[14],	// Right Leg

		Rotations[12],	// Left Leg Up 
		Rotations[12],	// Left Leg
		Rotations[13],	// Left Foot 

		// Left Side Leg
		Rotations[13],	// Left Leg
		Rotations[4],		// Left Shoulder
		Rotations[5],		// Left Arm

		Rotations[13],	// Left Leg
		Rotations[4],		// Left Shoulder 
		Rotations[16],	// Right Leg

		Rotations[13],	// Left Leg
		Rotations[12],	// Left Leg Up
		Rotations[15],	// Right Leg Up

		Rotations[13],	// Left Leg
		Rotations[14],	// Left Foot 
		Rotations[17],	// Right Foot

		// Left Side Ankle
		Rotations[14],	// Left Foot
		Rotations[4],		// Left Shoulder
		Rotations[8],	// Right Shoulder

		Rotations[14],	// Left Foot
		Rotations[13],	// Left Leg
		Rotations[16],	// Right Leg

		// Cross Center
		Rotations[1],		// Head
		Rotations[9],	// Right Elbow
		Rotations[14],	// Left Foot

		Rotations[1],		// Head
		Rotations[5],		// Left Elbow
		Rotations[17],	// Right Foot

		Rotations[1],		// Head
		Rotations[9],	// Right Elbow
		Rotations[6],	// Left Wrist

		Rotations[1],		// Head
		Rotations[5],		// Left Elbow
		Rotations[10],	// Right WRist

		Rotations[3],		// Spine 1
		Rotations[15],	// Right Hip
		Rotations[6],	// Left Wrist

		Rotations[3],		// Spine 1
		Rotations[12],	// Left Hip
		Rotations[10],	// Right Wrist

		Rotations[3],		// Spine 1
		Rotations[8],	// Right Shoulder
		Rotations[6],	// Left Wrist

		Rotations[3],		// Spine 1
		Rotations[4],		// Left Shoulder
		Rotations[10],	// Right Wrist

		Rotations[15],	// Right Up Leg 
		Rotations[4],		// Left Shoulder
		Rotations[9],	// Right Arm

		Rotations[12],	// Left Leg Up 
		Rotations[8],	// Right Shoulder
		Rotations[5],		// Left Arm

		Rotations[15],	// Right Up Leg 
		Rotations[4],		// Left Shoulder
		Rotations[10],	// Right Wrist


		Rotations[12],	// Left Leg Up 
		Rotations[8],	// Right Shoulder
		Rotations[6],	// Left Wrist

		Rotations[15],	// Right Up Leg 
		Rotations[5],		// Left Arm
		Rotations[10],	// Right Wrist

		Rotations[12],	// Left Leg Up 
		Rotations[9],	// Right Arm
		Rotations[6],	// Left Wrist

		Rotations[15],	// Right Up Leg 
		Rotations[13],	// Left Leg
		Rotations[10],	// Right Wrist

		Rotations[12],	// Left Leg Up 
		Rotations[16],	// Right Leg
		Rotations[6],	// Left Wrist

		Rotations[15],	// Right Up Leg 
		Rotations[14],	// Left Foot
		Rotations[16],	// Right Leg

		Rotations[12],	// Left Leg Up 
		Rotations[17],	// Right Root
		Rotations[13],	// Left Leg

		Rotations[16],	// Right Leg
		Rotations[4],	// Left Shoulder
		Rotations[9],	// Right Arm

		Rotations[13],	// Left Leg
		Rotations[8],	// Right Shoulder
		Rotations[5]	// Left Arm


	};
}
void FJointBufferThread::UpdateDebugLines()
{

	// Update debug vector arrays
	int TriangleCount = 0;
	for (int i = 0; i < TrianglePositions.Num() / 3; i++) {
		/*
		Directional vector is D1 normalized
		T is the midpoint of the side of the triangle
		<rx,ry,rz> -> slopes of the line / Parallel Directional Vector
		t is any parameter (real value) will give us a point on the line
		x = x0 + rx * t
		y = y0 + ry * t
		z = z0 + rz * t

		For AB
		x = ABmid.x + D1.x * t
		y = ABmid.y + D1.y * t
		z = ABmid.z + D1.z * t

		For BC
		x = BCmid.x + D2.x * s
		y = BCmid.y + D2.y * s
		z = BCmid.z + D2.z * s

		For CA
		x = CAmid.x + D3.x * q
		y = CAmid.y + D3.y * q
		z = CAmid.z + D3.z * q

		Set each component equal to each other

		ABBC
		ABmid.x + D1.x * t = BCmid.x + D2.x * s
		ABmid.y + D1.y * t = BCmid.y + D2.y * s
		ABmid.z + D1.z * t = BCmid.z + D2.z * s

		BCCA
		BCmid.x + D2.x * s = CAmid.x + D3.x * q
		BCmid.y + D2.y * s = CAmid.y + D3.y * q
		BCmid.z + D2.z * s = CAmid.z + D3.z * q

		CAAB
		CAmid.x + D3.x * q = ABmid.x + D1.x * t
		CAmid.y + D3.y * q = ABmid.y + D1.y * t
		CAmid.z + D3.z * q = ABmid.z + D1.z * t

		Solve for s
		s = ( ABmid.x + D1.x * t - BCmid.x ) / ( D2.x )
		Subsititute into the y equation
		ABmid.y + D1.y * t = BCmid.y + D2.y * ( ( ABmid.x + D1.x * t - BCmid.x ) / ( D2.x ) )    // Substitution
		D1.y * t - D2.y * ( ( ABmid.x + D1.x * t - BCmid.x ) / ( D2.x ) ) = BCmid.y - ABmid.y    // Bring T to the same side
		( D1.y * t ) - ( D2.y * ABmid.x + D2.y * D1.x * t - D2.y * BCmid.x ) / ( D2.x ) = ( BCmid.y - ABmid.y ) // Expand
		( D1.y * t ) * ( D2.x ) - ( D2.y * ABmid.x + D2.y * D1.x * t - D2.y * BCmid.x ) = ( BCmid.y - ABmid.y ) * ( D2.x ) // Multiply out the divisor
		( D1.y * t ) * ( D2.x ) - ( D2.y * D1.x * t ) =  ( BCmid.y - ABmid.y ) * ( D2.x ) + ( D2.y * ABmid.x ) - ( D2.y * BCmid.x )  // Add
		t ( D1.y * D2.x - D2.y * D1.x ) = ( BCmid.y - ABmid.y ) * ( D2.x ) + ( D2.y * ABmid.x ) - ( D2.y * BCmid.x )  // Factor
		t = ( ( BCmid.y - ABmid.y ) * ( D2.x ) + ( D2.y * ABmid.x ) - ( D2.y * BCmid.x ) ) / ( D1.y * D2.x - D2.y * D1.x )  // Divide

		PVector ABBCx = new PVector();

		ABBCx.x = ABmid.x + D1.x * ( ( ( BCmid.y - ABmid.y ) * ( D2.x ) + ( D2.y * ABmid.x ) - ( D2.y * BCmid.x ) ) / ( D1.y * D2.x - D2.y * D1.x ) );
		ABBCx.y = ABmid.y + D1.y * ( ( ( BCmid.y - ABmid.y ) * ( D2.x ) + ( D2.y * ABmid.x ) - ( D2.y * BCmid.x ) ) / ( D1.y * D2.x - D2.y * D1.x ) );
		ABBCx.z = ABmid.z + D1.z * ( ( ( BCmid.y - ABmid.y ) * ( D2.x ) + ( D2.y * ABmid.x ) - ( D2.y * BCmid.x ) ) / ( D1.y * D2.x - D2.y * D1.x ) );
		*/

		int StartIndex = TrianglePositions.Num() - 201;

		FVector A(TrianglePositions[StartIndex + TriangleCount * 3].X, TrianglePositions[StartIndex + TriangleCount * 3].Y, TrianglePositions[StartIndex + TriangleCount * 3].Z);
		FVector B(TrianglePositions[StartIndex + TriangleCount * 3 + 1].X, TrianglePositions[StartIndex + TriangleCount * 3 + 1].Y, TrianglePositions[StartIndex + TriangleCount * 3 + 1].Z);
		FVector C(TrianglePositions[StartIndex + TriangleCount * 3 + 2].X, TrianglePositions[StartIndex + TriangleCount * 3 + 2].Y, TrianglePositions[StartIndex + TriangleCount * 3 + 2].Z);

		FVector _AB = A - B;
		FVector _ABmid((A.X + B.X) / 2, (A.Y + B.Y) / 2, (A.Z + B.Z) / 2);
		FVector _BC = B - C;
		FVector _BCmid((B.X + C.X) / 2, (B.Y + C.Y) / 2, (B.Z + C.Z) / 2);
		FVector _CA = C - A;
		FVector _CAmid((C.X + A.X) / 2, (C.Y + A.Y) / 2, (C.Z + A.Z) / 2);

		FVector _V = FVector::CrossProduct(_AB, _BC);
		FVector _D1 = FVector::CrossProduct(_V, _AB);
		FVector _D2 = FVector::CrossProduct(_V, _BC);
		FVector _D3 = FVector::CrossProduct(_V, _CA);

		_V.Normalize();

		_V = _V * 50;

		_D1.Normalize();
		_D2.Normalize();
		_D3.Normalize();

		_D1 = _D1 * 150;
		_D2 = _D2 * 150;
		_D3 = _D3 * 150;

		AB.Emplace(_AB);
		ABmid.Emplace(_ABmid);
		BC.Emplace(_BC);
		BCmid.Emplace(_BCmid);
		CA.Emplace(_CA);
		CAmid.Emplace(_CAmid);

		V.Emplace(_V);
		D1.Emplace(_D1);
		D2.Emplace(_D2);
		D3.Emplace(_D3);

		float CircumcenterX = _ABmid.X + _D1.X * (((_BCmid.Y - _ABmid.Y) * (_D2.X) + (_D2.Y * _ABmid.X) - (_D2.Y * _BCmid.X)) / (_D1.Y * _D2.X - _D2.Y * _D1.X));
		float CircumcenterY = _ABmid.Y + _D1.Y * (((_BCmid.Y - _ABmid.Y) * (_D2.X) + (_D2.Y * _ABmid.X) - (_D2.Y * _BCmid.X)) / (_D1.Y * _D2.X - _D2.Y * _D1.X));
		float CircumcenterZ = _ABmid.Z + _D1.Z * (((_BCmid.Y - _ABmid.Y) * (_D2.X) + (_D2.Y * _ABmid.X) - (_D2.Y * _BCmid.X)) / (_D1.Y * _D2.X - _D2.Y * _D1.X));
		FVector _ABBC = FVector(CircumcenterX, CircumcenterY, CircumcenterZ);
		ABBC.Emplace(_ABBC);
		TriangleCount++;
	}

	if (AB.Num() > TriangleCount) {
		for (int i = 0; i < AB.Num() - TriangleCount; i++) {
			AB.RemoveAt(i, 1, true);
			ABmid.RemoveAt(i, 1, true);
			BC.RemoveAt(i, 1, true);
			BCmid.RemoveAt(i, 1, true);
			CA.RemoveAt(i, 1, true);
			CAmid.RemoveAt(i, 1, true);

			V.RemoveAt(i, 1, true);
			D1.RemoveAt(i, 1, true);
			D2.RemoveAt(i, 1, true);
			D3.RemoveAt(i, 1, true);
			ABBC.RemoveAt(i, 1, true);
		}
	}
}

void FJointBufferThread::UpdateEulerLines()
{
	EulerLines.Empty();
	TriangleCircumcenters.Empty();
	TriangleCentroids.Empty();
	// Update debug vector arrays
	int TriangleCount = 0;
	for (int i = 0; i < TrianglePositions.Num() / 3; i++) {

		int StartIndex = TrianglePositions.Num() - 201;

		FVector A(TrianglePositions[StartIndex + TriangleCount * 3].X, TrianglePositions[StartIndex + TriangleCount * 3].Y, TrianglePositions[StartIndex + TriangleCount * 3].Z);
		FVector B(TrianglePositions[StartIndex + TriangleCount * 3 + 1].X, TrianglePositions[StartIndex + TriangleCount * 3 + 1].Y, TrianglePositions[StartIndex + TriangleCount * 3 + 1].Z);
		FVector C(TrianglePositions[StartIndex + TriangleCount * 3 + 2].X, TrianglePositions[StartIndex + TriangleCount * 3 + 2].Y, TrianglePositions[StartIndex + TriangleCount * 3 + 2].Z);

		FVector Centroid((A.X + B.X + C.X) / 3, (A.Y + B.Y + C.Y) / 3, (A.Z + B.Z + C.Z) / 3);
		double _a[3] = { double(A.X), double(A.Y), double(A.Z) };
		double _b[3] = { double(B.X), double(B.Y), double(B.Z) };
		double _c[3] = { double(C.X), double(C.Y), double(C.Z) };
		double _result[3] = { 0,0,0 };
		TriCircumCenter3D(_a, _b, _c, _result);
		FVector Circumcenter = FVector(_result[0], _result[1], _result[2]);
		FVector Final = Centroid - Circumcenter;
		TriangleCentroids.Emplace(Centroid);
		TriangleCircumcenters.Emplace(Circumcenter);
		EulerLines.Emplace(Final);
		TriangleCount++;
	}

	//UE_LOG(LogTemp, Display, TEXT("Euler Lines Created"));
}

void FJointBufferThread::UpdateTracking()
{

	SmoothingSamplesCount = 3;
	if (TriangleTauBuffers.size() == 0) {
		int TriangleCount = 0;
		for (int i = 0; i < TriangleIndexBoneNames.Num() / 3; i++) {
			FString Base = TriangleIndexBoneNames[TriangleCount * 3].ToString().Append(TriangleIndexBoneNames[TriangleCount * 3 + 1].ToString()).Append(TriangleIndexBoneNames[TriangleCount * 3 + 2].ToString());
			FName Name = FName(*Base);
			//UE_LOG(LogTemp, Display, TEXT("Tracking tau for %i"), TriangleCount);
			std::ostringstream s;
			s << TriangleCount;
			std::string CountIndex = s.str();
			std::string SName(TCHAR_TO_UTF8(*Name.ToString()));
			std::string BufferName = "Triangle Tau Buffer " + CountIndex + SName;
			FString FBufferName(BufferName.c_str());
			FName FinalName = FName(*FBufferName);
			//UE_LOG(LogTemp, Display, TEXT("%s"), *FinalName.ToString());
			int StartIndex = TrianglePositions.Num() - 201;
			FVector A(TrianglePositions[StartIndex + TriangleCount * 3].X, TrianglePositions[StartIndex + TriangleCount * 3].Y, TrianglePositions[StartIndex + TriangleCount * 3].Z);
			FVector Circumcenter = TriangleCircumcenters[i];
			FVector Radius = TrianglePositions[0];
			FVector EulerLine = EulerLines[i];
			UTauBuffer* TriangleBuffer = new UTauBuffer();
			TriangleBuffer->CurrentTime = FApp::GetCurrentTime();
			TriangleBuffer->BeginningTime = FApp::GetCurrentTime();
			TriangleBuffer->LastMeasuringStick = TriangleBuffer->MeasuringStick;
			TriangleBuffer->MeasuringStick = Radius;
			TriangleBuffer->BeginningPosition = FVector4(EulerLine.X, EulerLine.Y, EulerLine.Z, 0.0);
			TriangleBuffer->MotionPath.Emplace(TriangleBuffer->BeginningPosition);
			TriangleTauBuffers.emplace_back(TriangleBuffer);
			TriangleCount++;

		}
		return;
	}

	//UE_LOG(LogTemp, Warning, TEXT("Triangle tau buffers length: %i"), TriangleTauBuffers.Num());

	int TriangleCount = 0;
	for (int i = 0; i < TriangleTauBuffers.size(); i++) {
		FName Name = TriangleIndexBoneNames[TriangleCount * 3];
		//UE_LOG(LogTemp, Display, TEXT("Tracking tau for %i"), TriangleCount);
		UTauBuffer* Buffer = TriangleTauBuffers[TriangleCount];
		//UE_LOG(LogTemp, Display, TEXT("%s"), *Buffer->GetFName().ToString());
		Buffer->LastReadingTime = Buffer->CurrentTime;
		Buffer->CurrentTime = FApp::GetCurrentTime();
		//UE_LOG(LogTemp, Display, TEXT("Check Time %f, %f"),Buffer->CurrentTime, Buffer->LastReadingTime);
		Buffer->ElapsedSinceLastReadingTime = FApp::GetDeltaTime();
		Buffer->ElapsedSinceBeginningGestureTime = Buffer->CurrentTime - Buffer->BeginningTime;
		//UE_LOG(LogTemp, Display, TEXT("Elapsed Time %f"), Buffer->ElapsedSinceLastReadingTime);
		Buffer->ElapsedTimeSamples.Emplace(Buffer->ElapsedSinceLastReadingTime);
		FVector EulerLine = EulerLines[i];
		Buffer->MotionPath.Emplace(FVector4(EulerLine.X, EulerLine.Y, EulerLine.Z, Buffer->CurrentTime));
		Buffer->EndingPosition = FVector4(EulerLine.X, EulerLine.Y, EulerLine.Z, Buffer->CurrentTime);
		Buffer->CalculateIncrementalGestureChange(i);
		//Buffer->CalculateFullGestureChange();

		if (Buffer->IncrementalAngleTauSamples.Num() > SmoothingSamplesCount)
		{
			Buffer->IncrementalAngleTauSamples.RemoveAt(0, 1, true);
		}
		if (Buffer->IncrementalAngleTauDotSamples.Num() > SmoothingSamplesCount)
		{
			Buffer->IncrementalAngleTauDotSamples.RemoveAt(0, 1, true);
		}
		if (Buffer->IncrementalAngleTauDotSmoothedDiffFromLastFrame.Num() > SmoothingSamplesCount)
		{
			Buffer->IncrementalAngleTauDotSmoothedDiffFromLastFrame.RemoveAt(0, 1, true);
		}
		if (Buffer->IncrementalPositionTauSamples.Num() > SmoothingSamplesCount)
		{
			Buffer->IncrementalPositionTauSamples.RemoveAt(0, 1, true);
		}
		if (Buffer->IncrementalPositionTauDotSamples.Num() > SmoothingSamplesCount)
		{
			Buffer->IncrementalPositionTauDotSamples.RemoveAt(0, 1, true);
		}
		if (Buffer->IncrementalPositionTauDotSmoothedDiffFromLastFrame.Num() > SmoothingSamplesCount)
		{
			Buffer->IncrementalPositionTauDotSmoothedDiffFromLastFrame.RemoveAt(0, 1, true);
		}
		if (Buffer->ElapsedTimeSamples.Num() > SmoothingSamplesCount) {
			Buffer->ElapsedTimeSamples.RemoveAt(0, 1, true);
		}
		if (Buffer->IncrementalGestureAngleChanges.Num() > SmoothingSamplesCount) {
			Buffer->IncrementalGestureAngleChanges.RemoveAt(0, 1, true);
		}
		if (Buffer->IncrementalGesturePositionChanges.Num() > SmoothingSamplesCount) {
			Buffer->IncrementalGesturePositionChanges.RemoveAt(0, 1, true);
		}
		if (Buffer->MotionPath.Num() > SmoothingSamplesCount) {
			Buffer->MotionPath.RemoveAt(0, 1, true);
		}
		if (Buffer->FullGestureTauSamples.Num() > SmoothingSamplesCount)
		{
			Buffer->FullGestureTauSamples.RemoveAt(0, 1, true);
		}
		if (Buffer->FullGestureTauDotSamples.Num() > SmoothingSamplesCount)
		{
			Buffer->FullGestureTauDotSamples.RemoveAt(0, 1, true);
		}
		if (Buffer->FullGestureTauDotSmoothedDiffFromLastFrame.Num() > SmoothingSamplesCount)
		{
			Buffer->FullGestureTauDotSmoothedDiffFromLastFrame.RemoveAt(0, 1, true);
		}

		if (false) {
			int index = 0;
			for (double Sample : Buffer->IncrementalAngleTauSamples)
			{
				UE_LOG(LogTemp, Display, TEXT("Tau Angle Samples: %i\t%f"), index, Sample);
				index++;
			}
			index = 0;
			for (double Sample : Buffer->IncrementalPositionTauSamples)
			{
				UE_LOG(LogTemp, Display, TEXT("Tau Position Samples: %i\t%f"), index, Sample);
				index++;
			}
			index = 0;
			for (double Sample : Buffer->IncrementalAngleTauDotSamples)
			{
				UE_LOG(LogTemp, Display, TEXT("Tau Angle Dot Samples: %i\t%f"), index, Sample);
				index++;
			}
			index = 0;
			for (double Sample : Buffer->IncrementalPositionTauDotSamples)
			{
				UE_LOG(LogTemp, Display, TEXT("Tau Position Dot Samples: %i\t%f"), index, Sample);
				index++;
			}

		}

		TriangleCount++;
	}
}