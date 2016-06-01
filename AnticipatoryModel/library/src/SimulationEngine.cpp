/*
 *  SimulationEngine.cpp
 *  
 *  
 *  All rights are retained by the authors and the University of Minnesota.
 *  Please contact sjguy@cs.umn.edu for licensing inquiries.
 *  
 *  Authors: Ioannis Karamouzas, Brian Skinner, and Stephen J. Guy
 *  Contact: ioannis@cs.umn.edu
 */


#include "SimulationEngine.h"

#if _OPENMP
#include <omp.h>
#endif


TTC::SimulationEngine* simEngine;

namespace TTC {

	Vector2D zero(Vector2D, double t){
		return Vector2D(0,0);
	}

	SimulationEngine::SimulationEngine()
	{
		_spatialDatabase = NULL;
		simEngine = this;

	}

	SimulationEngine::~SimulationEngine()
	{

		for(vector<Agent*>::iterator it = _agents.begin(); it != _agents.end(); ++it)
		{
			delete *it;
			*it = 0x0;

		}
		
		for(vector<LineObstacle*>::iterator it = _obstacles.begin(); it != _obstacles.end(); ++it)
		{
			delete *it;
			*it = 0x0;
		}
		
			
		if (_spatialDatabase!=NULL)
		{
			delete _spatialDatabase;
			_spatialDatabase = 0x0;
		}

	}

	void SimulationEngine::init(float xRange, float yRange)
	{
		_iteration = 0;
		_globalTime = 0;
		_reachedGoals = false;
		
		//initialize the database
		_spatialDatabase = new SpatialProximityDatabase(Vector2D(), Vector2D(xRange,yRange), Vector2D(10.0f, 10.0f));
		//ForceFunction = zero;
		
	}


	void SimulationEngine::updateSimulation()
	{
		_reachedGoals = true;

		#pragma omp parallel for
		for (int i = 0; i < (int)_agents.size(); ++i)
		{
			if (_agents[i]->enabled())
			{
				_agents[i]->doStep();
				_reachedGoals = false;
			}
		}


		#pragma omp parallel for
		for (int i = 0; i < (int)_agents.size(); ++i)
		{
			if (_agents[i]->enabled())
			{
				_agents[i]->update();
			}
		}

		_globalTime += _dt;
		_iteration++;
	}


	bool SimulationEngine::outsideRoom(Agent* agent){
		return false;
	}

	bool SimulationEngine::allAreDeadOrEvacuated(){
		for(int i = 0; i < (int)_agents.size(); i++){
			if(_agents[i]->isDead() || outsideRoom(_agents[i])){

			}else{
				return false;
			}
		}
		return true;
	}	

	bool SimulationEngine::endSimulation()
	{

		return _reachedGoals || _iteration >= _maxSteps || allAreDeadOrEvacuated();
	}

	void SimulationEngine::updateVisualisation()
	{
	  // Output the current global time. 
		std::cout << "Time: " << _globalTime << std::endl;

	  // Output the current position of all the agents. 
	  for (unsigned int i = 0; i < _agents.size(); ++i)
		  if (_agents[i]->enabled())
			  std::cout << i << ": " <<_agents[i]->position() << " ";
	  std::cout << std::endl;
	}
	
	void SimulationEngine::printCSVHeader(){
		std::cout << "time;";

		for(int i = 0 ; i < _agents.size()-1; i++){
			std::cout << i << ";";
		}

		if(_agents.size())
			std::cout << _agents.size()-1 << std::endl;
	}

	void SimulationEngine::printCSV(){
		// std::cout << "printCSV " << force(Vector2D(0,0)) << std::endl;
		std::cout << _globalTime << ";";

		for(int i = 0 ; i < _agents.size()-1; i++){
			std::cout << _agents[i]->position() << ";";
		}

		if(_agents.size()){
			std::cout << _agents[_agents.size()-1]->position() << std::endl;
		}
	}
	
	void SimulationEngine::exportSimulation(std::ofstream& file)
	{   
		for (unsigned int i = 0; i < _agents.size(); ++i)
		{
			if (_agents[i]->enabled())
				file << _agents[i]->id() << ","  << _agents[i]->position().x << "," << _agents[i]->position().y <<"," << 
				_agents[i]->radius() << "," << _globalTime <<std::endl;

		}
	
	}

	void SimulationEngine::addAgent(AgentInitialParameters& agentConditions)
	{
		// std::cout << "addAgent " << force(Vector2D(0,0)) << std::endl;

		Agent* newAgent = new Agent();
		if (newAgent != NULL) {
			newAgent->init(agentConditions);
			// std::cout << "addAgent " << force(Vector2D(0,0)) << std::endl;
			newAgent->setForceFunction(this->force);
			newAgent->setRadiusFunction(this->rad);
			_agents.push_back(newAgent);
		}
	}

	void SimulationEngine::setForceFunction(ForceFunction force){
		// std::cout << "SimulationEngine.cpp " << force(Vector2D(0,0)) << std::endl;
		this->force = force;
		// std::cout << "SimulationEngine.cpp " << this->force(Vector2D(0,0)) << std::endl;
	}

	void SimulationEngine::setRadiusFunction(RadiusFunction rad){
		this->rad = rad;
	}

	void SimulationEngine::addObstacle(const std::pair<Vector2D, Vector2D>& lineSegment)
	{
		LineObstacle* l = new LineObstacle(lineSegment.first, lineSegment.second);
		_obstacles.push_back(l);
	}

}