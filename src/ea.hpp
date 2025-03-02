#pragma once

#include "drone.hpp"
#include "net.hpp"
#include <SFML/Graphics/BlendMode.hpp>
#include <SFML/Graphics/CircleShape.hpp>
#include <SFML/System/Vector2.hpp>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <functional>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>

using Individual = std::unique_ptr<Net>;
using Agent = std::unique_ptr<Drone>;

constexpr float HALF_PI = M_PI * 0.5f;

struct EA {
	std::vector<sf::Vector2f> goals;
	std::vector<sf::CircleShape> walls;
	std::vector<Agent> agents;
	bool simFinished = false;

	EA(size_t popSize, const Net &mother, const Drone &father)
		: popSize(popSize) {
		assert(popSize % 2 == 0 && "PopSize should be divisible by 2!");

		population.reserve(popSize);
		populationW.reserve(popSize);
		agents.reserve(popSize);
		fitness.resize(popSize);

		initPop(mother);
		initAgents(father);
		simFinished = false;

		// original having a problem with sides
		/* goals = {sf::Vector2f{200, 200}, */ 
		/* 	sf::Vector2f{200, 600}, */
		/* 	sf::Vector2f{600, 600}, */ 
		/* 	sf::Vector2f{600, 200}, */
		/* 	sf::Vector2f{400, 400}}; */

	}

	void update(const float dt, const sf::Vector2f &boundary, bool debug=false) {
		sf::Vector2f goalDist;
		std::vector<float> observation;
		Output output;

		bool someAlive = false;

		Drone* drone;
		Net* net;

		for (int i = 0; i < popSize; ++i) {
			drone = agents[i].get();
			net = population[i].get();

			drone->update(dt, boundary);

			if (!drone->alive) continue;
			someAlive = true;

			goalDist = goals[drone->goalIndex % goals.size()] - drone->pos;
			observation = {
				drone->vel.x / 20.0f,
				drone->vel.y / 20.0f,
				(drone->angle) / HALF_PI,
				goalDist.x / 800,
				goalDist.y / 800
			};

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

			float expGx = exp(-abs(goalDist.x)/800);
			float expGy = exp(-abs(goalDist.y)/800);

			fitness[i] += (drone->goalIndex+1)*(expGx + expGy);

			if (debug) {
				if (i == 0) {
					std::cout << "DEBUG:" << std::endl;
					std::cout << "GD: " << goalDist.x/800 << "," << goalDist.y/800 << std::endl;
					std::cout << "FGD: " << (drone->goalIndex+1)*(expGx + expGy) << std::endl;
				}
			}

			output = net->predict(observation);
			assert(output.size() == 4 && "Drone expects 4 net outputs");
			drone->control(output[0], output[1], output[2], output[3]);
		}

		if (!someAlive) {
			simFinished = true;
		}
	}

	void process() {
		auto elite = fitnessAgents();
		std::vector<Weights> eliteW;
		for (auto i : elite) {
			eliteW.push_back(populationW[i]);
		}

		/* std::cout << "EA PROCESS" << std::endl; */
		auto selectedIds = tournamentSelection();
		/* std::cout << "Tournament done" << std::endl; */
		auto offspringWeights = crossover(selectedIds);
		/* std::cout << "Crossover done" << std::endl; */
		mutation(offspringWeights);
		/* std::cout << "Mutation done" << std::endl; */

		populationW = offspringWeights;

		for (int i = 0; i < eliteW.size(); ++i) {
			populationW[i] = eliteW[i];
		}
		/* std::cout << "Elite size: " << eliteW.size() << std::endl; */

		resetAgents();
		/* std::cout << "Agents reseted" << std::endl; */
		simFinished = false;
	}

  private:
	void initPop(const Net &mother) {
		for (int i = 0; i < popSize; ++i) {
			population.push_back(std::make_unique<Net>());

			for (const auto & mod : mother.modules) {
				population[i]->modules.push_back(mod->clone());
			}

			population[i]->initialize();
			populationW.push_back(population[i]->getWeights());
		}
	}

	// return an elite vector
	std::vector<size_t> fitnessAgents() {
		std::vector<size_t> eliteIds;

		for (int i = 0; i < popSize; ++i) {
			fitness[i] += 1000 * agents[i]->goalIndex;
		}

		const int eliteSize = popSize*0.05;
		std::vector<float> sortedFitness(eliteSize); //largest n numbers
		std::partial_sort_copy(
			std::begin(fitness), std::end(fitness), 
			std::begin(sortedFitness), std::end(sortedFitness), 
			std::greater() 
		);

		/* auto max = std::max_element(fitness.begin(), fitness.end()); */
		/* int argmax = std::distance(fitness.begin(), max); */
		std::cout << "MaxFitness: " << sortedFitness[0] << std::endl;

		for (auto sf : sortedFitness) {
			size_t i = std::find(fitness.begin(), fitness.end(), sf) - fitness.begin();
			eliteIds.push_back(i);
		}
		return eliteIds;
	}

	void initAgents(const Drone &father) {
		for (int i = 0; i < popSize; ++i) {
			agents.push_back(std::make_unique<Drone>(father.startPos));
		}
	}

	void resetAgents() {
		for (int i = 0; i < popSize; ++i) {
			agents[i]->reset();
			population[i]->loadWeights(populationW[i]);
			fitness[i] = 0;
		}
	}

	std::vector<size_t> tournamentSelection() {
		std::uniform_int_distribution<std::mt19937::result_type> distr(0, popSize - 1);

		std::vector<size_t> selectedIds;
		selectedIds.reserve(popSize);

		const int tournamentSize = 2;

		size_t bestId;
		float bestFitness;
		size_t id;
		for (int i = 0; i < popSize; ++i) {
			// running the tournament
			bestId = -1;
			bestFitness = std::numeric_limits<float>::min();
			for (int x = 0; x < tournamentSize; ++x) {
				id = distr(gen);
				if (fitness[id] > bestFitness) {
					bestFitness = fitness[id];
					bestId = id;
				}
			}

			selectedIds[i] = bestId;
		}

		return selectedIds;
	}

	std::vector<Weights> crossover(const std::vector<size_t> &selectedIds) {
		std::uniform_real_distribution<float> distr(0.0f, 1.0f);

		std::vector<Weights> newPopW;
		newPopW.reserve(popSize);

		for (int i = 0; i < popSize; i += 2) {
			Weights p1 = populationW[selectedIds[i]];
			Weights p2 = populationW[selectedIds[i + 1]];

			Weights o1;
			Weights o2;
			o1.reserve(p1.size());
			o2.reserve(p2.size());

			for (int k = 0; k < p1.size(); ++k) {
				if (distr(gen) < 0.5) {
					o1.push_back(p1[k]);
					o2.push_back(p2[k]);
				} else {
					o1.push_back(p2[k]);
					o2.push_back(p1[k]);
				}
			}

			newPopW.push_back(o1);
			newPopW.push_back(o2);
		}

		return newPopW;
	}

	void mutation(std::vector<Weights> &offspringW) {
		std::uniform_real_distribution<float> weightDistr(-1.0f, 1.0f);
		std::uniform_real_distribution<float> chanceDistr(0.0f, 1.0f);

		const float MUTPROB = 0.03;

		for (int i = 0; i < popSize; ++i) {
			for (int k = 0; k < offspringW[i].size(); ++k) {
				if (chanceDistr(gen) < MUTPROB) {
					offspringW[i][k] = weightDistr(gen);
				}
			}
		}
	}

	std::vector<Individual> population;
	std::vector<Weights> populationW;
	std::vector<float> fitness;

	const size_t popSize;

private: 
	void scenario_default() {
		this->goals = {
			sf::Vector2f{200, 200}, 
			sf::Vector2f{600, 600}, 
			sf::Vector2f{200, 600},
			sf::Vector2f{600, 200},
			sf::Vector2f{400, 400}
		};
		this->walls = {};
	}

	void scenario_default_with_walls() {
		this->goals = {
			sf::Vector2f{200, 200}, 
			sf::Vector2f{600, 600}, 
			sf::Vector2f{200, 600},
			sf::Vector2f{600, 200},
			sf::Vector2f{400, 400}
		};
	}
};
