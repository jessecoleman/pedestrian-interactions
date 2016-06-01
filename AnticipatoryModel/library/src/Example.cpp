/*
 *  Example.cpp
 *  
 *  
 *  All rights are retained by the authors and the University of Minnesota.
 *  Please contact sjguy@cs.umn.edu for licensing inquiries.
 *  
 *  Authors: Ioannis Karamouzas, Brian Skinner, and Stephen J. Guy
 *  Contact: ioannis@cs.umn.edu
 */
 

/*!
 *  @file       Example.cpp
 *  @brief      An example of how to use this library.
 */
#include "SimulationEngine.h"
#include <utility>
using namespace TTC;

double radiusGrowth = 0;


SimulationEngine * _engine = 0;

void destroy ()
{
	delete _engine; 
	_engine = 0x0;
}

double radius(double t){
	return radiusGrowth * t;
}

Vector2D force(Vector2D f, double t){


	if(f.x == 0 || f.y == 0)
		return Vector2D(0,0);

	double fireTemp = 1;
	double fireFalloff = 1;

	double r = radius(t);
	double distFromCircle = f.length() - r;

	return f / f.length() * fireTemp * exp(-fireFalloff * distFromCircle);

}

void setupScenario()
{


	//initialize the engine, given the dimensions of the environment
	_engine->init(50, 50);

    // Specify the default parameters for agents that are subsequently added.	
	AgentInitialParameters par;

	par.k = 1.5f;
	par.ksi = 0.54f;
	par.m = 2.0f;
	par.t0 = 3.f;
	par.neighborDist = 10.f;
	par.maxAccel = 20.f; 
	par.radius = 0.5f;
	par.prefSpeed = 1.4f;
	par.goalRadius = 1.5f;
	
 
	// Add agents, specifying their start positions, goal positions and preferred speeds. The agents are distributed along the circumference of a circle
	// and have to move to their antipodal positions.
   for (int i = 0; i < 10; ++i) 
	{
		AgentInitialParameters p = par;  
		p.position = 10.f * Vector2D(cos(i * 2.0f * _M_PI / 10.0f), sin(i * 2.0f * _M_PI / 10.0f));
		p.goal = Vector2D(-p.position.x, -p.position.y);
		p.velocity = Vector2D();
		//gaussian distributed speed
		float u;
		do{
			u = (float)rand()/(float)RAND_MAX;
		} while (u >= 1.0);
		p.prefSpeed += sqrtf( -2.f * logf( 1.f - u)) * 0.1f * cosf(2.f*_M_PI*(float)rand()/(float)RAND_MAX);
		_engine->addAgent(p);
   }

   
   /* Static obstacles can be added as follows:
	*   std::pair<Vector2D, Vector2D> lineSegment = std::make_pair(Vector2D(-20, -20), Vector2D(20, -20));
	*   _engine->addObstacle(lineSegment);
    */
}

void twoPersonScenario(){

	_engine->init(50, 50);

    // Specify the default parameters for agents that are subsequently added.	
	AgentInitialParameters par;

	par.k = 1.5f;
	par.ksi = 0.54f;
	par.m = 2.0f;
	par.t0 = 3.f;
	par.neighborDist = 10.f;
	par.maxAccel = 20.f; 
	par.radius = 0.5f;
	// par.prefSpeed = 1.4f;
	par.goalRadius = 1.5f;

	AgentInitialParameters person1 = par;
	AgentInitialParameters person2 = par;

	person1.position = Vector2D(-10, 0.0001);
	person2.position = Vector2D(10, 0);

	person1.goal = Vector2D(10, 0);
	person2.goal = Vector2D(-10, 0);

	person1.velocity = Vector2D();
	person2.velocity = Vector2D();

	person1.prefSpeed = 2;
	person2.prefSpeed = 2;

	_engine->addAgent(person1);
	_engine->addAgent(person2);

}
 
void bottleneckScenario(){
	_engine->init(50, 50);
	_engine->setForceFunction(force);

    // Specify the default parameters for agents that are subsequently added.	
	AgentInitialParameters par;

	par.k = 1.5f;
	par.ksi = 0.54f;
	par.m = 2.0f;
	par.t0 = 3.f;
	par.neighborDist = 10.f;
	par.maxAccel = 20.f; 
	par.radius = 0.5f;
	par.prefSpeed = 1.4f;
	par.goalRadius = 1.5f;

	//make box
	Vector2D ulcorner(-4, 8.001);
	Vector2D urcorner(4,8.001);
	Vector2D llcorner(-4,-8.001);
	Vector2D lrcorner(4,-8.001);
	Vector2D upperdoor(4,2);
	Vector2D lowerdoor(4,-2);

	std::pair<Vector2D, Vector2D> topline(ulcorner, urcorner);
	std::pair<Vector2D, Vector2D> leftline(ulcorner, llcorner);
	std::pair<Vector2D, Vector2D> bottomline(llcorner, lrcorner);
	std::pair<Vector2D, Vector2D> toprightline(urcorner, upperdoor);
	std::pair<Vector2D, Vector2D> bottomrightline(lrcorner, lowerdoor);

	_engine->addObstacle(topline);
	_engine->addObstacle(leftline);
	_engine->addObstacle(bottomline);
	_engine->addObstacle(toprightline);
	_engine->addObstacle(bottomrightline);

   for (int i = 0; i < 50; i ++){
		AgentInitialParameters p = par;  
		p.position = Vector2D(((float)rand()/(float)RAND_MAX-0.5)*4,((float)rand()/(float)RAND_MAX-0.5)*8);
		p.goal = Vector2D(10, 0);
		p.velocity = Vector2D();

		//gaussian distributed speed
		float u;
		do{
			u = (float)rand()/(float)RAND_MAX;
		} while (u >= 1.0);
		p.prefSpeed += sqrtf( -2.f * logf( 1.f - u)) * 0.1f * cosf(2.f*_M_PI*(float)rand()/(float)RAND_MAX);
		_engine->addAgent(p);
   }
}


void twoPersonScenarioForce(){
	// std::cout << "example.cpp " << force(Vector2D(0,0)) << std::endl;
	_engine->setForceFunction(force);
	_engine->setRadiusFunction(radius);
	twoPersonScenario();
}

void bottleneckScenarioForce(){
	_engine->setForceFunction(force);
	_engine->setRadiusFunction(radius);

	bottleneckScenario();
}

void twoDoorBottleneckScenarioForce(){
	_engine->init(50, 50);
	_engine->setForceFunction(force);
	_engine->setRadiusFunction(radius);


    // Specify the default parameters for agents that are subsequently added.	
	AgentInitialParameters par;

	par.k = 1.5f;
	par.ksi = 0.54f;
	par.m = 2.0f;
	par.t0 = 3.f;
	par.neighborDist = 10.f;
	par.maxAccel = 20.f; 
	par.radius = 0.5f;
	par.prefSpeed = 1.4f;
	par.goalRadius = 1.5f;

	//make box
	Vector2D ulcorner(-4, 8.001);
	Vector2D urcorner(4,8.001);
	Vector2D llcorner(-4,-8.001);
	Vector2D lrcorner(4,-8.001);
	Vector2D Rupperdoor(4,2);
	Vector2D Rlowerdoor(4,-2);
	Vector2D Lupperdoor(-4, 2);
	Vector2D Llowerdoor(-4,-2);

	std::pair<Vector2D, Vector2D> topline(ulcorner, urcorner);
	std::pair<Vector2D, Vector2D> bottomline(llcorner, lrcorner);
	std::pair<Vector2D, Vector2D> toprightline(urcorner, Rupperdoor);
	std::pair<Vector2D, Vector2D> bottomrightline(lrcorner, Rlowerdoor);
	std::pair<Vector2D, Vector2D> topleftline(ulcorner, Lupperdoor);
	std::pair<Vector2D, Vector2D> bottomleftline(llcorner, Llowerdoor);


	_engine->addObstacle(topline);
	_engine->addObstacle(bottomline);
	_engine->addObstacle(toprightline);
	_engine->addObstacle(bottomrightline);
	_engine->addObstacle(topleftline);
	_engine->addObstacle(bottomleftline);

   for (int i = 0; i < 50; i ++){
		AgentInitialParameters p = par;  
		p.position = Vector2D(((float)rand()/(float)RAND_MAX-0.5)*4,((float)rand()/(float)RAND_MAX-0.5)*8);
		Vector2D goal1(10,0);
		Vector2D goal2(-10,0);

		if((p.position - goal1).length() > (p.position - goal2).length()){
			p.goal = goal2;
		}else{
			p.goal = goal1;
		}
		
		p.velocity = Vector2D();

		//gaussian distributed speed
		float u;
		do{
			u = (float)rand()/(float)RAND_MAX;
		} while (u >= 1.0);
		p.prefSpeed += sqrtf( -2.f * logf( 1.f - u)) * 0.1f * cosf(2.f*_M_PI*(float)rand()/(float)RAND_MAX);
		_engine->addAgent(p);
   }
}

// int main(int argc, char **argv)
// {	
// 	//seed randomly
// 	srand (time(NULL));

// 	//default parameters
// 	int numFrames = 700;
// 	float dt = 0.05f;
	
// 	//load the engine
// 	// std::cout << "creating engine" << std::endl;
// 	_engine = new SimulationEngine(); 
// 	_engine->setTimeStep(dt);
// 	_engine->setMaxSteps(numFrames);

// 	// setup the scenario
// 	// std::cout << "setting up scenario" << std::endl;
// 	bottleneckScenarioForce();
	
// 	_engine->printCSVHeader();
// 	// Run the scenario
// 	do 
// 	{
// 		// std::cout << "printCSV" << std::endl;
// 		_engine->printCSV();
// 		// std::cout << "update" << std::endl;
// 		// std::cout << "numdead " << _engine->numberDead() << " done: " << _engine->numberDone() << " agents " << _engine->getAgents().size() <<std::endl;
// 		_engine->updateSimulation();
// 	} while ( !_engine->endSimulation());	

// 	//destroy the environment
// 	destroy();

// 	return 0;	
// }

int numberDead;
double evacTime; 

void simulate(){
	//default parameters
	int numFrames = 700;
	float dt = 0.05f;
	
	//load the engine
	// std::cout << "creating engine" << std::endl;
	_engine = new SimulationEngine(); 
	_engine->setTimeStep(dt);
	_engine->setMaxSteps(numFrames);

	// setup the scenario
	// std::cout << "setting up scenario" << std::endl;
	twoDoorBottleneckScenarioForce();
	
	// _engine->printCSVHeader();
	// Run the scenario
	do 
	{
		_engine->updateSimulation();
	} while ( !_engine->endSimulation());	

	numberDead = _engine->numberDead();
	evacTime = _engine->getGlobalTime();

	//destroy the environment
	destroy();
}

int main(){
	srand (time(NULL));
	std::cout << "alpha, numberDead, evacTime" << std::endl;
	for(double alpha = 0; alpha < 2.5; alpha += 0.1){
		for(int i = 0; i < 10; i++){
			radiusGrowth = alpha;
			simulate();
			std::cout << alpha <<  "," << numberDead << "," << evacTime << std::endl;
		}
	}

}




