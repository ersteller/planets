
#include "Quadtree.h"

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
/* 1. make round static bodys to a bin */
/* 2. tidal force or impact should break bins (testcase: check roche border) d= 2.423 * R * (rohM/rohm)^(1/3)*/

	char* Tree::getAncestors(char* buff)
	{
		Tree* w = this;
		int d = GetDepth() * 3;
		memcpy(&buff[0], " ro", 3);
		while (w->parent != NULL)
		{
			if(w == w->parent->nw)
				memcpy(&buff[d], "_nw", 3);
			else if(w == w->parent->ne)
				memcpy(&buff[d], "_ne", 3);		
			else if(w == w->parent->sw)
				memcpy(&buff[d], "_sw", 3);
			else if(w == w->parent->se)
				memcpy(&buff[d], "_se", 3);
			else 
				assert(0);
			d-=3;
			assert(d>=0);
			w = w->parent;
		}	 
		return buff;
	}

	// k error between 0 and 1 
	// if lager than 1 we are only evaluating root 
	// if k == 0 we use direct method
	float32 Tree::GetK(b2Vec2* cur)
	{
		// spread of box
		b2Vec2 direction;
		float32 radius = ((midY - minY) * 2);
	    direction.x = midX - cur->x;
		direction.y = midY - cur->y;
		float32 length = abs(direction.x) + abs(direction.y); // direction.Length(); // this takes 10%of all time direction.Length(); 


		// k is an indicator for expectet error impact 
		// sensable between 0 and 1 if lager than 1 we are very close to or inside this node
		float32 k = 0;
		if (length != 0)
			k = radius / length;
		else 
			k = std::numeric_limits<float>::max();		
		return k;
	}


	/*Tree* oldNext(float32 kerror, b2Vec2* cur)
	{
		Tree* ret = NULL;
		Tree* worker = this;
		return NULL;
	}*/


	/// find next node after this one (sequence is nw --> ne --> sw --> se --> parent-parent)
	Tree* Tree::Next(float32 kerror, b2Vec2* cur)
	{	
		Tree* ret = NULL;
		Tree* worker = this;

		/* use short circuit before calling GetK */
		if (worker->hasChildren && worker->GetK(cur) > kerror )
		{
			while (worker->hasChildren && worker->GetK(cur) > kerror)
			{
				// we should go deeper
				worker = worker->nw;
			}
			// we are now at the correct level
			// skip border nodes
			if (worker->nSubParticles != 0 || worker->nParticles != 0)
				ret = worker;
			else 
				ret = worker->Next(kerror,cur);
			return ret;
		}

		while (worker->parent != NULL && worker == worker->parent->se )
		{
			// at the end of children go up
			worker = worker->parent; 
			// and one more next to the side 
		}
				
		// get next of same layer 
		if (worker->parent != NULL)
		{
			if(worker == worker->parent->nw)
				worker = worker->parent->ne;
			else if(worker == worker->parent->ne)
				worker = worker->parent->sw;			
			else if(worker == worker->parent->sw)
				worker = worker->parent->se;
			else 
				assert(0); // did not find itself in the children of parent
			

			while (worker->hasChildren && worker->GetK(cur) > kerror)
			{
				// we should go deeper
				worker = worker->nw;
			}
			// we are now at the correct level
			// skip border nodes
			if (worker->nSubParticles != 0 || worker->nParticles != 0)
				ret = worker;
			else 
				ret = worker->Next(kerror,cur);
			return ret;


		}
		// we arrived in root node 
		else 
			return NULL;	
	}

	void Tree::UpdateCenterOfMass(b2Vec2* cur, float32 CurMass)
	{
		float32 NewMass = Mass + CurMass;

		/* arithmetic middle incrementally  */
		CenterOfMass.x = CenterOfMass.x * Mass / NewMass + cur->x * CurMass / NewMass;
		CenterOfMass.y = CenterOfMass.y * Mass / NewMass + cur->y * CurMass / NewMass;
		Mass = NewMass;
	}

	Tree::HandleAndPos* Tree::add(b2Vec2 * cur, const b2ParticleHandle * phP, Tree::HandleAndPos *ptHPin) {
		// TODO: maybe optimizable
		HandleAndPos* pRes = NULL;
		
		if(hasChildren) 
		{
			if(cur->x < midX) 
			{
				if(cur->y < midY) 
				{
					pRes = nw->add(cur, phP, ptHPin);
				} 
				else 
				{
					pRes = sw->add(cur, phP, ptHPin);
				}
			} 
			else 
			{
				if(cur->y < midY) 
				{
					pRes = ne->add(cur, phP, ptHPin);
				} 
				else 
				{
					pRes = se->add(cur, phP, ptHPin);
				}
			}
			// add the current particle to the subparticle list because we have added it to the child
			nSubParticles++;
			subparticles.push_back(pRes);
			
			/* update center of mass */
			if(ptHPin == NULL)
			{  // so this is a new/unknown particle (not moved to children from parent)
				float32 CurMass = 1; //TODO: mass of this particle may be individuel
				UpdateCenterOfMass(cur, CurMass);
			}
		} 
		else 
		{
			if(nParticles < maxParticles) {
				if (ptHPin == NULL)
				{										
					ptHPin = new HandleAndPos;
					ptHPin->pos = cur;
					ptHPin->handle = (b2ParticleHandle*)phP;
				}
				pRes = ptHPin;
				particles.push_back(ptHPin); 
				nParticles++;
				
				{ 
				    float32 CurMass = 1; //TODO: mass of this particle may be individuel
				    UpdateCenterOfMass(cur, CurMass);
			    }

				
			} else {

				if(nw == NULL)
				{
					nw = new Tree(minX, minY, midX, midY, this);
					ne = new Tree(midX, minY, maxX, midY, this);
					sw = new Tree(minX, midY, midX, maxY, this);
					se = new Tree(midX, midY, maxX, maxY, this);
				}
				else
				{ 
					/* only update positions*/
					nw->minX = minX;
					nw->minY = minY;
					nw->maxX = midX;
					nw->maxY = midY;
					nw->setMid();

					ne->minX = midX;
					ne->minY = minY;
					ne->maxX = maxX;
					ne->maxY = midY;
					ne->setMid();

					sw->minX = minX;
					sw->minY = midY;
					sw->maxX = midX;
					sw->maxY = maxY;
					sw->setMid();

					se->minX = midX;
					se->minY = midY;
					se->maxX = maxX;
					se->maxY = maxY;
					se->setMid();
				}
				hasChildren = true;

				/* TODO: Binning here somewhere */

				/* move particles to children*/
				HandleAndPos* ptHP = NULL;
				std::deque<HandleAndPos*>::iterator it;
				while(nParticles > 0)
				{
					it = particles.begin();
					ptHP = it[0];
					// calling add again but now we have children and pointer
					add(ptHP->pos, ptHP->handle, ptHP);

					// We keep track of all subparticles when we have children 
					particles.pop_front();
					nParticles--;
				}
				assert(nParticles == 0);
				// finally adding the current particle now reentering to add to correct child 
				pRes = add(cur, phP);
			}
		}
		return pRes; // return pointer of partricle for subparticle list of parents
	}

	void Tree::setup(b2Vec2* all, int32 iParticleCount) 
	{
		int n = iParticleCount;
		if(n > 0) 
		{
			// find boundaries
			minX = all[0].x;
			minY = all[0].y;
			maxX = minX;
			maxY = minY;

			for(int i = 0; i < n; i++) 
			{
				if(all[i].x < minX)
					minX = all[i].x;
				if(all[i].y < minY)
					minY = all[i].y;
				if(all[i].x > maxX)
					maxX = all[i].x;
				if(all[i].y > maxY)
					maxY = all[i].y;
			}

			// center and square boundaries
			setMid();
			float diffX = maxX - minX;
			float diffY = maxY - minY;
			float halfSide = std::max(diffX, diffY) / 2;
			minX = midX - halfSide;
			maxX = midX + halfSide;
			minY = midY - halfSide;
			maxY = midY + halfSide;

			//draw(); //debugdraw
		}
	}

	/*cleanup tree 
	particlereferences are removed from tree
	nodes without particles and children (border cleanup)
	delets particle list 
	keeps children for later use if they just had particles or subparticles */
	void Tree::cleanup()
	{
		if (hasChildren)
		{
			nw->cleanup();
			ne->cleanup();
			sw->cleanup();
			se->cleanup();
		}
		else if(nw != NULL)
		{ // in this node no children were required 
			delete nw;
			delete ne;
			delete sw;
			delete se;
			nw = NULL;
			ne = NULL;
			sw = NULL;
			se = NULL;
		}

		delParticles();
		hasChildren = false;

		subparticles.clear();
		nSubParticles = 0;

		CenterOfMass.x = 0.f;
		CenterOfMass.y = 0.f;
		Mass = 0; 

		destroyBox();
	}

    /* this is a debug function; 
	it creates the b2bodys around edges of this leef of the 
	tree called from drawonlythis */
	void Tree::makeBox()
	{
		b2BodyDef bd;
		m_debugBody = m_world->CreateBody(&bd);
		m_debugBody->SetActive(false);
		b2ChainShape shape;
		const b2Vec2 vertices[4] = {
			b2Vec2(minX, minY),
			b2Vec2(maxX, minY),
			b2Vec2(maxX, maxY),
			b2Vec2(minX, maxY)};
		shape.CreateLoop(vertices, 4);
		m_debugBody->CreateFixture(&shape, 0.0f); 
	}
	
    /* this is a debug function; 
	it draws a debugbodys around the edges of the tree recursifly */
	void Tree::draw(b2World* m_world) {
		this->m_world = m_world;
		destroyBox();
		if(nParticles)
			makeBox();
		
		//ofRect(minX, minY, maxX - minX, maxY - minY);
		if(hasChildren) {
			nw->draw( m_world);
			ne->draw( m_world);
			sw->draw( m_world);
			se->draw( m_world);
		}
	}
