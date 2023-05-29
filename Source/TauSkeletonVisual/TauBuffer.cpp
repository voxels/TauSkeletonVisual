// Fill out your copyright notice in the Description page of Project Settings.


#include "TauBuffer.h"
#include "Kismet/KismetMathLibrary.h"

// Sets default values for this component's properties
UTauBuffer::UTauBuffer()
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	IsAngleGrowing = false;
	IsPositionGrowing = false;
	MeasuringStick = FVector(0);
	LastMeasuringStick = FVector(0);
	BeginningPosition = FVector(0);
	EndingPosition = FVector(0);

	MotionPath.Emplace(BeginningPosition);

	BeginningTime = FApp::GetCurrentTime();
	LastReadingTime = FApp::GetCurrentTime();
	CurrentTime = FApp::GetCurrentTime();

	ElapsedSinceLastReadingTime = 0;
	ElapsedSinceBeginningGestureTime = 0;
}


void UTauBuffer::CalculateIncrementalGestureChange(int index)
{
	bool DebugLog = false; /* (index == 3);*/
	if (MotionPath.Num() < 2) {
		return;
	}

	FVector4 BeginningNormal = MotionPath.Last(1);
	FVector4 EndingNormal = MotionPath.Last();

	FVector PositionChange = EndingNormal - BeginningNormal;
	IncrementalGesturePositionChanges.Emplace(PositionChange);
	BeginningNormal = BeginningNormal.GetSafeNormal();
	EndingNormal = EndingNormal.GetSafeNormal();

	if (DebugLog) {
		UE_LOG(LogTemp, Display, TEXT("End Normal: X:%f Y:%f Z:%f W:%f"), EndingNormal.X, EndingNormal.Y, EndingNormal.Z, EndingNormal.W);
		UE_LOG(LogTemp, Display, TEXT("Beginning Normal: X:%f Y:%f Z:%f W:%f"), BeginningNormal.X, BeginningNormal.Y, BeginningNormal.Z, BeginningNormal.W);
	}
	float CurrentGestureDotProduct = (BeginningNormal.X * EndingNormal.X) + (BeginningNormal.Y * EndingNormal.Y) + (BeginningNormal.Z * EndingNormal.Z);	
	float AngleChange = UKismetMathLibrary::Acos(CurrentGestureDotProduct);
	if (DebugLog) {
		UE_LOG(LogTemp, Display, TEXT("Angle Change: %f"), AngleChange);
	}
	IncrementalGestureAngleChanges.Emplace(AngleChange);

	//UE_LOG(LogTemp, Display, TEXT("Incremental gesture angle changes: %i"), IncrementalGestureAngleChanges.Num());
	//UE_LOG(LogTemp, Display, TEXT("Elapsed time samples: %i"), ElapsedTimeSamples.Num());
	if (IncrementalGestureAngleChanges.Num() >= 2 && ElapsedTimeSamples.Num() >= 2) {
		double EndTime = ElapsedTimeSamples.Last();

		if (EndTime > 0) {

			double AverageAngleChange = 0;
			double AverageTimeElapsed = 0;
			for (double IncrementalAngleChange : IncrementalGestureAngleChanges)
			{
				AverageAngleChange += IncrementalAngleChange;
			}
			for (double ElapsedTime : ElapsedTimeSamples)
			{
				AverageTimeElapsed += ElapsedTime;
			}

			AverageAngleChange = AverageAngleChange / IncrementalGestureAngleChanges.Num();
			AverageTimeElapsed = AverageTimeElapsed / ElapsedTimeSamples.Num();
			double AverageVelocity = AverageAngleChange / AverageTimeElapsed;
			if ((AverageVelocity >= 0.1 || AverageVelocity <= -0.1) && (AngleChange >= 0.1 || AngleChange <= -0.1)) {
				double IncrementalTau = AngleChange / AverageVelocity;
				IncrementalAngleTauSamples.Emplace(IncrementalTau);
				if (DebugLog) {
					UE_LOG(LogTemp, Display, TEXT("End Angle: %f"), AngleChange);
					UE_LOG(LogTemp, Display, TEXT("Average Velocity: %f"), AverageVelocity);
					UE_LOG(LogTemp, Display, TEXT("Incremental Angle Time Check: %f"), EndTime);
					UE_LOG(LogTemp, Display, TEXT("Calculated Incremental Angle Tau: %f"), IncrementalTau);
				}
			}
			else {
				double IncrementalTau = 0;
				IncrementalAngleTauSamples.Emplace(IncrementalTau);
				if (DebugLog) {
					UE_LOG(LogTemp, Display, TEXT("Last Angle Change: %f"), AngleChange);
					UE_LOG(LogTemp, Display, TEXT("Average Velocity: %f"), AverageVelocity);
					UE_LOG(LogTemp, Display, TEXT("Incremental Angle Time Check: %f"), EndTime);
					UE_LOG(LogTemp, Display, TEXT("Calculated Incremental Angle Tau: %f"), IncrementalTau);
				}

			}
		}
		else {
			IncrementalAngleTauSamples.Emplace(0);
			if (DebugLog) {
				UE_LOG(LogTemp, Display, TEXT("Zero Incremental Angle Tau: %f"), 0);
			}
		}
	}

	if (IncrementalGesturePositionChanges.Num() >= 2 && ElapsedTimeSamples.Num() >= 2) {
		
		double EndTime = ElapsedTimeSamples.Last();

		if (EndTime > 0) {
			double PositionChangeDistance = FVector::Distance(EndingNormal, BeginningNormal);
			double AveragePositionChange = 0;
			double AverageTimeElapsed = 0;
			for (int ii = 1; ii < IncrementalGesturePositionChanges.Num(); ii++) {
				FVector EndVector = IncrementalGesturePositionChanges[ii];
				FVector StartVector = IncrementalGesturePositionChanges[ii - 1];
				double ThisChangeDistance = FVector::DistSquared(EndVector, StartVector);
				AveragePositionChange += ThisChangeDistance;
			}
			
			for (double ElapsedTime : ElapsedTimeSamples)
			{
				AverageTimeElapsed += ElapsedTime;
			}

			AveragePositionChange = AveragePositionChange / IncrementalGesturePositionChanges.Num();
			AverageTimeElapsed = AverageTimeElapsed / ElapsedTimeSamples.Num();
			double AverageVelocity = AveragePositionChange / AverageTimeElapsed;
			if ((AverageVelocity >= 0.1 || AverageVelocity <= -0.1) && (PositionChangeDistance >= 0.1 || PositionChangeDistance <= -0.1)) {
				double IncrementalTau = PositionChangeDistance / AverageVelocity;
				IncrementalPositionTauSamples.Emplace(IncrementalTau);
				if (DebugLog) {
					UE_LOG(LogTemp, Display, TEXT("Incremental Position Tau: %f"), IncrementalTau);
				}
			}
			else {
				double IncrementalTau = 0;
				IncrementalPositionTauSamples.Emplace(IncrementalTau);
				if (DebugLog) {
					UE_LOG(LogTemp, Display, TEXT("Incremental Position Tau: %f"), IncrementalTau);
				}
			}
		}
		else {
			//UE_LOG(LogTemp, Display, TEXT("Incremental Position Tau Time Check: %f"), EndTime);
		}
	}
	
	if (IncrementalAngleTauSamples.Num() > 2) {
		double EndTau = IncrementalAngleTauSamples.Last();
		double CheckTau = IncrementalAngleTauSamples.Last(1);

		double EndTime = ElapsedTimeSamples.Last();

		if (EndTime > 0) {
			double IncrementalTauDot = (EndTau - CheckTau) / (EndTime);
			IncrementalAngleTauDotSamples.Emplace(IncrementalTauDot);
			//UE_LOG(LogTemp, Display, TEXT("Angle Tau Dot Angle Check: %f, %f"), EndTau, CheckTau);
			//UE_LOG(LogTemp, Display, TEXT("Angle Tau Dot Time Check: %f"), EndTime );
			//UE_LOG(LogTemp, Display, TEXT("Incremental Angle Tau Dot: %f"), IncrementalTauDot);
			IsAngleGrowing = IncrementalTauDot >= 0.5;
		}
	}
	
	if (IncrementalPositionTauSamples.Num() > 2) {
		double EndTau = IncrementalPositionTauSamples.Last();
		double CheckTau = IncrementalPositionTauSamples.Last(1);

		double EndTime = ElapsedTimeSamples.Last();

		if (EndTime > 0) {
			double IncrementalTauDot = (EndTau - CheckTau) / (EndTime);
			IncrementalPositionTauDotSamples.Emplace(IncrementalTauDot);
			//UE_LOG(LogTemp, Display, TEXT("Incremental Position Tau Dot: %f"), IncrementalTauDot);
			IsPositionGrowing = IncrementalTauDot >= 0.5;
		}
	}

	//UE_LOG(LogTemp, Display, TEXT("Incremental Tau Dot Position Is Growing:\t%d"), IsPositionGrowing);
	//UE_LOG(LogTemp, Display, TEXT("Incremental Tau Dot Angle Is Growing:\t%d"), IsAngleGrowing);

	/*

	if (IncrementalAngleTauDotSamples.Num() > 4) {
		double EndTau = IncrementalAngleTauDotSamples.Last();
		double ThirdTau = IncrementalAngleTauDotSamples.Last(1);
		double SecondTau = IncrementalAngleTauDotSamples.Last(2);

		double SecondMean = (EndTau + ThirdTau) / 2;
		double FirstMean = (ThirdTau + SecondTau) / 2;
		double Diff = SecondMean - FirstMean;
		IncrementalAngleTauDotSmoothedDiffFromLastFrame.Emplace(Diff);

		//UE_LOG(LogTemp, Display, TEXT("Incremental Tau Dot Smoothed Mean: %f"), SecondMean);
	}

	if (IncrementalAngleTauDotSmoothedDiffFromLastFrame.Num() > 2) {
		double EndDiff = IncrementalAngleTauDotSmoothedDiffFromLastFrame.Last();
		double CheckDiff = IncrementalAngleTauDotSmoothedDiffFromLastFrame.Last(1);
		if (abs(EndDiff - CheckDiff) < EndDiff * 0.15) {
			//UE_LOG(LogTemp, Display, TEXT("Incremental Diff is constant:\t%f\t%f"), EndDiff, CheckDiff);
		}
		else
		{
			//UE_LOG(LogTemp, Display, TEXT("Diff is NOT constant:\t%f\t%f"), EndDiff, CheckDiff);
		}

		if (EndDiff - CheckDiff < 0) {
			IsGrowing = false;
		}
		else {
			IsGrowing = true;
		}

		//UE_LOG(LogTemp, Display, TEXT("Incremental Is Growing:\t%d"), IsGrowing);
	}

	*/
}