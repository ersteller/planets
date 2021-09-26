// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "MutualForceBpLib.generated.h"

#include "Quadtree.h"

/**
 * 
 * 
 */
UCLASS()
class PLANETS_API UMutualForceBpLib : public UBlueprintFunctionLibrary
{
    // public:
	GENERATED_BODY()
        UFUNCTION(BlueprintCallable, Category = "AaMy Nodes")
        static void AddForceFunction(UStaticMeshComponent* obj, FVector vect);
        
    // GENERATED_BODY()
 	    static void AddGroup(UStaticMeshComponent* particleSystem);
    // GENERATED_BODY()
 	    static void Step(/*const b2TimeStep& step*/);
 	UMutualForceBpLib();
 	~UMutualForceBpLib();



    //static void MutualForceFunction(UStaticMeshComponent* obj1, UStaticMeshComponent* obj2);




 //private:
 	int m_stepCount;
 	Tree* m_tree;
 	UStaticMeshComponent* m_particleSystem;
 	float m_radius;
 };
};

