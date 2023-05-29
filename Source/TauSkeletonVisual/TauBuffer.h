// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Math/Vector4.h"
#include "Math/Vector.h"


class UTauBuffer 
{

public:	
	// Sets default values for this component's properties
	UTauBuffer();

	bool IsPositionGrowing;

	bool IsAngleGrowing;

		bool FullGestureIsGrowing;

		FVector LastMeasuringStick;

		FVector MeasuringStick;

		FVector4 BeginningPosition;

		FVector4 EndingPosition;

		TArray<FVector4> MotionPath;

		TArray<float> FullGestureAngleChanges;

		TArray<float> IncrementalGestureAngleChanges;

		TArray<FVector4> IncrementalGesturePositionChanges;

		TArray<float> IncrementalAngleTauSamples;

		TArray<float> IncrementalPositionTauSamples;

		TArray<float> FullGestureTauSamples;

		TArray<float> IncrementalAngleTauDotSamples;

		TArray<float> IncrementalPositionTauDotSamples;

		TArray<float> FullGestureTauDotSamples;

		TArray<float> IncrementalAngleTauDotSmoothedDiffFromLastFrame;

		TArray<float> IncrementalPositionTauDotSmoothedDiffFromLastFrame;

		TArray<float> FullGestureTauDotSmoothedDiffFromLastFrame;

		double BeginningTime;

		double LastReadingTime;

		double CurrentTime;

		double ElapsedSinceLastReadingTime;

		double ElapsedSinceBeginningGestureTime;

		TArray<double> ElapsedTimeSamples;

		void CalculateIncrementalGestureChange(int index);
};
