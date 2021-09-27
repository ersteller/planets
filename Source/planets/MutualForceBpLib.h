// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "Quadtree.h"
#include "MutualForceBpLib.generated.h"


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
        void AddForceFunction(UStaticMeshComponent* obj, FVector vect);
        
    // GENERATED_BODY()
 	    void AddGroup(UStaticMeshComponent* particleSystem, int iParticleCount);
    // GENERATED_BODY()
 	    void Step(/*const b2TimeStep& step*/);
 	UMutualForceBpLib();
 	~UMutualForceBpLib();



    //static void MutualForceFunction(UStaticMeshComponent* obj1, UStaticMeshComponent* obj2);




 //private:
    int m_particleCount;
 	int m_stepCount;
 	Tree* m_tree;
 	UStaticMeshComponent* m_particleSystem;
 	float m_radius;

};

