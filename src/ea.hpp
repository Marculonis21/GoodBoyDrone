#pragma once

#include "drone.hpp"
#include "net.hpp"
#include "SFML/System/Vector2.hpp"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

using Individual = std::unique_ptr<Net>;
using Agent = std::unique_ptr<Drone>;

struct EAItem {
	Net *net; 
	Drone *drone; 
};

struct FitnessStats {
	float max = 0;
	float min = 0;
	float avg = 0;
	float med = 0;
};

struct AbstractEA {
	uint64_t generation = 0;
	size_t input_size;

	FitnessStats lastFitnessStats;

	AbstractEA(size_t popSize, const Net &mother, const Drone &father) : popSize(popSize), motherDescription(mother.describe())  {
		assert(popSize % 2 == 0 && "PopSize should be divisible by 2! (Please)");

		population.reserve(popSize);
		populationW.reserve(popSize);
		agents.reserve(popSize);
		fitness.resize(popSize);

		input_size = mother.input_size;
	}

	virtual ~AbstractEA() {}

	virtual const EAItem operator [](int idx) const {
		return EAItem{population[idx].get(), agents[idx].get()};
	}

	// base from EasyEA
	virtual bool update(const float dt, const World &world, bool debug=false) {
		std::vector<float> observation;
		observation.resize(input_size);

		Output output;

		bool someAlive = false;

		Drone* drone;
		Net* net;

		for (int i = 0; i < popSize; ++i) {
			drone = agents[i].get();
			net = population[i].get();

			drone->update(dt, world);

			if (!drone->alive) continue;
			someAlive = true;

			drone->genObservation_with_sensors(observation, world);
			/* drone->genObservation_no_sensors(observation, world); */

			// hard-coded goal collection
			sf::Vector2f goalDist = world.goals[drone->goalIndex % world.goals.size()] - drone->pos;
			if (goalDist.x*goalDist.x + goalDist.y*goalDist.y < 100) {
				drone->goalTimer += 1;
				// half a second for 60 fps game physics - GOAL COLLECTED
				if (drone->goalTimer > 30) {
					drone->goalTimer = 0;
					drone->goalIndex += 1;

					// reward for quickly obtaining the goal
					fitness[i] += (drone->goalIndex+1)*(600 - drone->aliveTimer);
					drone->aliveTimer *= 0.5f;
				}
			}
			else {
				drone->goalTimer = 0;
			}

			// fitness calculation
			float cosGx = cos((-goalDist.x/world.boundary.x) * M_PI/2.0f);
			float cosGy = cos((-goalDist.y/world.boundary.y) * M_PI/2.0f);
			cosGx = pow(cosGx, 4.0f);
			cosGy = pow(cosGy, 4.0f);
			// take the min because we want to penalize individuals going away
			float cosG = std::min(cosGx, cosGy);

			fitness[i] += (drone->goalIndex+1)*(cosG);

			if (debug) {
				if (i == 0) {
					std::cout << "DEBUG:" << std::endl;
					std::cout << "GD: " << goalDist.x/world.boundary.x << "," << goalDist.y/world.boundary.y << std::endl;
					std::cout << "FGD: " << (drone->goalIndex+1)*(cosG) << std::endl;
					std::cout << "Sensors: ["; 
					for (int i = 0; i < 8; ++i) {
						std::cout << observation[5+i] << ", ";
					}
					std::cout << "]" << std::endl;

					std::cout << "velx: " << observation[0] << ", vely: " << observation[1] << std::endl;
					std::cout << "cAngle: " << observation[2] << ", sAngle: " << observation[3] << std::endl;
					std::cout << "avel: " << observation[4] << std::endl;
				}
			}

			output = net->predict(observation);
			assert(output.size() == 4 && "Drone expects 4 net outputs");
			drone->control(output[0], output[1], output[2], output[3]);
		}

		// ask for process
		return !someAlive;
	}

	virtual void process() = 0;

	void saveEA(const std::string &path) const {
		assert(false && "NOT IMPLEMENTED");
	};
	static std::unique_ptr<AbstractEA> loadEA(const std::string &path) {
		assert(false && "NOT IMPLEMENTED");
		return 0;
	};

protected:
	std::vector<Agent> agents;
	std::vector<Individual> population;
	std::vector<Weights> populationW;
	std::vector<float> fitness;

	const size_t popSize;
	const json motherDescription;

	// base from EasyEA
	virtual void initPop(const Net &mother) {
		for (int i = 0; i < popSize; ++i) {
			population.push_back(std::make_unique<Net>());

			for (const auto & mod : mother.modules) {
				population[i]->modules.push_back(mod->clone());
			}

			population[i]->initialize();
			populationW.push_back(population[i]->getWeights());
		}
	}

	// base from EasyEA
	virtual void initAgents(const Drone &father) {
		for (int i = 0; i < popSize; ++i) {
			agents.push_back(std::make_unique<Drone>(father.startPos));
		}
	}

	// base from EasyEA
	virtual void resetAgents() {
		for (int i = 0; i < popSize; ++i) {
			agents[i]->reset();
			population[i]->loadWeights(populationW[i]);
			fitness[i] = 0;
		}
	}

	virtual std::vector<size_t> fitnessAgents() = 0;

	virtual void saveProcedure(const std::string &path) const = 0;
	virtual void loadProcedure(const std::string &path) = 0;
};
