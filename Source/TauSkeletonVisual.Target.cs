// Fill out your copyright notice in the Description page of Project Settings.

using UnrealBuildTool;
using System.Collections.Generic;

public class TauSkeletonVisualTarget : TargetRules
{
	public TauSkeletonVisualTarget(TargetInfo Target) : base(Target)
	{
		Type = TargetType.Game;
		bOverrideBuildEnvironment = true;
		bForceEnableExceptions = true;
		DefaultBuildSettings = BuildSettingsVersion.V2;

		ExtraModuleNames.AddRange( new string[] { "TauSkeletonVisual" } );
	}
}
