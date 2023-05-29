// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include <vector>
#include "CoreMinimal.h"
#include "HAL/Runnable.h"
#include "UObject/NameTypes.h" 

class FRunnableThread;
class UNuitrackSkeletonJointBuffer;
class UTauBuffer;

/**
 * 
 */
class FJointBufferThread : public FRunnable
{
public:
	FJointBufferThread(TArray<FName>_BoneNames, TArray<FVector>_Locations, TArray<FRotator>_Rotations, TArray<float>_Confidences, std::vector<UTauBuffer*> _PreviousTriangleTauBuffers, UNuitrackSkeletonJointBuffer* _JointBuffer);
	~FJointBufferThread();

	bool bStopThread;
	bool bProcessComplete;

		int MinDebugTriangleIndex;

		int MaxDebugTriangleIndex;

		int FrameCount;

		double LastReadingTime;

		int SmoothingSamplesCount;

		TArray<FVector> TrianglePositions;

		TArray<int> TriangleIndexes;

		TArray<FName> TriangleIndexBoneNames;

		TArray<FRotator> TriangleRotations;

		TArray<FVector> TriangleCentroids;

		TArray<FVector> TriangleCircumcenters;

		TArray<FVector> EulerLines;

		std::vector<UTauBuffer*> TriangleTauBuffers;

		void ProcessSocketRawData();

	virtual bool Init();
	virtual uint32 Run();
	virtual void Stop();

protected:
	TArray<FVector> AB;
	TArray<FVector> ABmid;
	TArray<FVector> BC;
	TArray<FVector> BCmid;
	TArray<FVector> CA;
	TArray<FVector> CAmid;
	TArray<FVector> V;
	TArray<FVector> D1;
	TArray<FVector> D2;
	TArray<FVector> D3;
	TArray<FVector> ABBC;

		void UpdateTriangles(TArray<FName>BoneNames, TArray<FVector>Locations, TArray<FRotator>Rotations);

		void UpdateDebugLines();

		void UpdateEulerLines();

		void UpdateTracking();

		float Map(float value,
			float istart,
			float istop,
			float ostart,
			float ostop);


private:

	TArray<FName> SocketBoneNames;
	TArray<FVector> SocketLocations;
	TArray<FRotator> SocketRotations;
	TArray<float> SocketConfidences;
	UNuitrackSkeletonJointBuffer *JointBuffer;
};
