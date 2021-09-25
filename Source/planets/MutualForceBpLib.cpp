// Fill out your copyright notice in the Description page of Project Settings.


#include "MutualForceBpLib.h"
#include "b2MutualForceController.h"


UMutualForceBpLib::UMutualForceBpLib()
{
	// constructor
	m_group = NULL;
	m_stepCount = 0;
	m_tree = new Tree;
	m_particleSystem = NULL;
  	// m_radius = 0.025f;
}


int UMutualForceBpLib::SomeIntReturningFunction(FString Text, int32 Number, FString& TextOut)
{
	int res = 12;
	return res;
}


// int UMutualForceBpLib::MutualForceFunction(UStaticMeshComponent* obj, FVector Force) {
void UMutualForceBpLib::AddForceFunction(UStaticMeshComponent* obj, FVector Force) {

	// add force to body
	obj->AddForce(Force, "None");
}

void UMutualForceBpLib::MutualForceFunction(UStaticMeshComponent* obj1, UStaticMeshComponent* obj2) {

	// calc force 
	float G = 1;
	float m1 = 1;
	float m2 = 1;

	float r = 700;


	
	
	float fForce = G * m1 * m2 / (r * r);

	FVector Force = {};


	obj1->AddForce(Force, "None");
	obj2->AddForce(-Force, "None");

}


void UMutualForceBpLib::Step(/*const b2TimeStep& step*/)
{

    int32 iParticleCount =  m_particleSystem->GetParticleCount();	
	b2Vec2* paHeadPos = m_particleSystem->GetPositionBuffer();
	b2ParticleColor* paColor = m_particleSystem->GetColorBuffer();
    
    
    b2Vec2 vPos;
    b2Vec2 vDistance;
    b2Vec2 vForce;
    float32 fltForce;
    float32 fltDistance;

    b2Vec2 CoM;
    float32 mass;
    
    // Constants 
    //float32 fltMass = 1;
    float32 fltOurG = 0.00002f;

    // once every steps
    if (this->m_stepCount % 1 == 0)
    {	
        m_tree->cleanup(); 
        m_tree->setup(paHeadPos, iParticleCount);
        //printf(" ################## add to tree ");
        for (int32 idx = 0; idx < iParticleCount; idx++)
        {
            //printf("%d ",idx);
            
            // printf("now particle add %"PRIx64"\n", (void*)&paHeadPos[idx]);
            m_tree->add(&paHeadPos[idx], m_particleSystem->GetParticleHandleFromIndex(idx));
        }
        //printf("\n\n\n\n");
        // m_tree->draw(m_world);
    }

    for (int32 idx = 0; idx < iParticleCount; idx++)
    {
        // particle of interest
        vPos = paHeadPos[idx];
        //paColor[idx].Set(255,255,255,128);
        vForce = b2Vec2(0.0f, 0.0f);
        float32 k = 0.5f;

        //printf("\n %d with \n",idx);

        int iterations = 0;
        // float32 seenmass = 0;  // debug help
        for (Tree* it = m_tree->iterator()->Next(k,&vPos);
            it != NULL;
            it = it->Next(k,&vPos))
        {
            //printf("    d:%d X:%f Y:%f K:%f \n",it->GetDepth() ,it->midX, it->midY, it->GetK(&vPos));
            

            // highlight all nodes of first particle
            /*if (idx == 0)
            {
                char szAncestors[200] = {0};
                it->drawonlythis(m_world);

                printf("idx %d iteration %d treeid %d %s\n",idx,iterations,  (int)it , it->getAncestors(szAncestors));
            }*/

            iterations++;

            mass = it->Mass;
            //seenmass +=mass; // debug help
            CoM = it->getCenterOfMass();

            vDistance = CoM - vPos;
            fltDistance = vDistance.Length();
            
            if ( fltDistance < m_radius/2)
            {

                // this is too cloose to exist must be touching forces or self
                //printf(" ### skipping distance %f idx %d iteration %d treeid %d\n",fltDistance,idx,iterations,  (int)it );
                continue;
            }

            vDistance.Normalize();

            fltForce = fltOurG * mass / (fltDistance * fltDistance) ; //b2Dot(vDistance,vDistance)
            vForce += vDistance * fltForce;	
        }

        //printf("idx %d seenmass %f\n",idx, seenmass);

        /*printf("  %f Force \n",vForce.Length());*/

        /*float32 force = vForce.Length();
        if (force > 0.1)
        {
              // color high force
              paColor[idx].Set(200,10,10,250);
        }
        else 
        {
            paColor[idx].Set( (uint8)(255*(1-force)), (uint8)(255*(1-force)), (uint8)(255*(1-force)),(uint8)(255*(1-force)));
        }*/

        // aply sum of all forces to particle
        m_particleSystem->ParticleApplyForce( idx, vForce);	
        //printf("iterations %d\n\n",iterations);
    }
}


void UMutualForceBpLib::AddGroup(b2ParticleSystem* particleSystem)
{
    this->m_particleSystem = particleSystem;
}
