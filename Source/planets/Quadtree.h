#pragma once
#include <limits>
#include <deque>

#include "MutualForceBpLib.generated.h"

//#include "b2Controller.h"
// #include "../../../Box2D/Box2D/Box2D.h"

//#define max(x,y) x>=y?x:y

/* this should be usable like:
adding particles 
use iterator(curpos, qoutient) of tree and let tree decide which node you get

*/

/* 
stolen from https://code.google.com/p/kyle/
	addons and demos for openframework
 maxParticles describes how many particles are allowed
 in each leaf. a normal quadtree only has 1 particle per
 leaf. this can cause issues if two particles are very close
 together, as it causes a large number of subdivisions.
 in an evenly distributed system, if maxParticles has a high
 value, the tree will approximate a binning system.
*/

/* TODO: define qoutient s/d (spread / distance) to conditionally use above*/

/* TODO: binning: find large bodies of connectect particles */
/* found the GrowParticleContactBuffer function. mabe we can use that to find particle blobs */
/* blobs may become solid groups. but not sure about how to transition and when maybe 
   depends on material (viskosity:whater never but rock maybe) */
/* 1. make round static bodys to a bin / blob they are propably static for some time */
/* 2. tidal force or impact should break bins to blobs (testcase: check roche border) d= 2.423 * R * (rohM/rohm)^(1/3)*/

/* there should be a module: mutual gravity controller that implements a step which was originally done in the testbed 
it should contain all parameters and maybe have some defined interface with a userdefined 
 callback function maybe for each particle debugging */

#define maxParticles 1

class Tree
{
public:
	
	struct HandleAndPos
	{
		void*          handle;
		FVector*       pos;
	};

	int nParticles;
	int nSubParticles;
	std::deque<HandleAndPos*> particles;
	std::deque<HandleAndPos*> subparticles;
	std::deque<HandleAndPos*>* ptmpparticles;

	float32 Mass;

	bool hasChildren;
	Tree *nw, *ne, *sw, *se; 
	Tree *parent;
	float minX, minY, midX, midY, maxX, maxY, minZ, maxZ;

	/* used for debug rendering tree */
	// b2World* m_world;     
	// b2Body* m_debugBody;  // todo 
	
	Tree() :
		nParticles(0),
		nSubParticles(0),
		hasChildren(false),
		CenterOfMass(0.f,0.f),
		Mass(0),
		parent(NULL),
		m_debugBody(NULL),
		nw(NULL),ne(NULL),
		sw(NULL),se(NULL),
		minX(0), minY(0),
		midX(0), midY(0),
		maxX(0), maxY(0) {
	}
	Tree(float _minX, float _minY, float _maxX, float _maxY, Tree* pparent) :
		nParticles(0),
		nSubParticles(0),
		hasChildren(false),
		CenterOfMass(0.f,0.f),
		Mass(0),
		parent(pparent),
		m_debugBody(NULL),
		nw(NULL),ne(NULL),
		sw(NULL),se(NULL),
		minX(_minX), minY(_minY),
		maxX(_maxX), maxY(_maxY) {
		setMid();
	}

	// not in step() please 
	~Tree() {
		
		delParticles();
		if(hasChildren || nw != NULL) {
			delete nw; 
			delete ne;
			delete sw;
			delete se;
			nw = NULL;
			ne = NULL;
			sw = NULL;
			se = NULL;
		}
	}

	void setMid() {
		midX = (minX + maxX) / 2;
		midY = (minY + maxY) / 2;
	}

	/*FVector* GetNewDirection(FVector* ptOrigin)//
	{
		FVector* ptRes = new FVector;

		// direction
		ptRes->x = midX - ptOrigin->x;
		ptRes->y = midY - ptOrigin->y;
		return ptRes;
	}*/


	Tree* iterator()
	{
		if(parent)
			return parent->iterator(); // returns root pointer
		else
			return this;
	}

	int GetDepth()
	{
		Tree* w = this;
		int d = 0;
		while (w->parent != NULL)
		{
			d++;
			w = w->parent;
		}
		return d;
	}

	char* getAncestors(char* buff);

	// k error between 0 and 1 
	// if lager than 1 we are only evaluating root 
	// if k == 0 we use direct method
	float32 GetK(FVector* cur);
	
	/// find next node after this one (sequence is nw --> ne --> sw --> se --> parent-parent)
	Tree* Next(float32 kerror, FVector* cur);

	std::deque<HandleAndPos*>* GetParticles()
	{	// member of the tree updatet with each add function
		if (hasChildren)
		{ 
			if (!subparticles.size())
			{ // try to avoid collecting child particles too many times
				assert(0); // we should have suparticles traked by add function
			}
			return &subparticles;
		}
		else
			return &particles;
	}

	void UpdateCenterOfMass(FVector* cur, float32 CurMass);

	/* add psrticle to the tree */
	HandleAndPos* add(FVector * cur, void* phP, HandleAndPos* ptHPin = NULL);

	/* init measurements */
	void setup(FVector* all, int32 iParticleCount); 

	/* delete all storage structures in the particle queue */
	void delParticles()
	{
		if (nParticles > 0)
		{
			for (std::deque<HandleAndPos*>::iterator it = particles.begin();
					it != particles.end();
					it++ )
			{
				delete it[0];
			}
			particles.clear();
			nParticles = 0;
		}
	}

	/*cleanup tree 
	particlereferences are removed from tree
	nodes without particles and children (border cleanup)
	delets particle list 
	keeps children for later use if they just had particles or subparticles */
	void cleanup();

	/* this is a debug function; 
	it draws a single debugbody around the edges of this leef of the tree */
	/*void drawonlythis(b2World* m_world) {
		this->m_world = m_world;
		//printf("%f" ,(this->maxX - this->minX ) / (this->maxY - this->minY)); 
		destroyBox();
		makeBox();
	}*/

	/* this is a debug function; 
	it draws a debugbodys around the edges of the tree recursifly */
	/*void draw(b2World* m_world);*/	

	FVector getCenterOfMass(){ return CenterOfMass;}
	
private: 

	FVector CenterOfMass;

	/* this is a debug function; 
	it creates the b2bodys around edges of this leef of the 
	tree called from drawonlythis */
	/*void makeBox();*/

	/* this is a debug function; 
	it deletes the debug body created with makebox called from drawonlythis */
	/*void destroyBox()
	{
		if(m_debugBody)
		{
			m_world->DestroyBody(m_debugBody);
			m_debugBody = NULL;
		}
	}*/

};

