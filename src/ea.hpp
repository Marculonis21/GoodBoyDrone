#pragma once

#include "drone.hpp"
#include "net.hpp"
#include "SFML/System/Vector2.hpp"
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

struct EA {
	std::vector<Agent> agents;
	std::vector<Individual> population;

	float lastMaxFitness = 0;
	float lastAverageFitness = 0;

	uint64_t generation = 0;

	EA(size_t popSize, const Net &mother, const Drone &father) : popSize(popSize), motherDescription(mother.describe())  {
		assert(popSize % 2 == 0 && "PopSize should be divisible by 2! (Please)");

		population.reserve(popSize);
		populationW.reserve(popSize);
		agents.reserve(popSize);
		fitness.resize(popSize);

		initPop(mother);
		initAgents(father);
	}

	bool update(const float dt, const World &world, bool debug=false) {
		// WARN: observation size is important for memory!
		std::vector<float> observation;
		observation.resize(13);

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
			float expGx = exp(-abs(goalDist.x)/world.boundary.x);
			float expGy = exp(-abs(goalDist.y)/world.boundary.y);

			fitness[i] += (drone->goalIndex+1)*(expGx + expGy);

			if (debug) {
				if (i == 0) {
					std::cout << "DEBUG:" << std::endl;
					std::cout << "GD: " << goalDist.x/world.boundary.x << "," << goalDist.y/world.boundary.y << std::endl;
					std::cout << "FGD: " << (drone->goalIndex+1)*(expGx + expGy) << std::endl;
					std::cout << "Sensors: ["; 
					for (int i = 0; i < 8; ++i) {
						std::cout << observation[5+i] << ", ";
					}
					std::cout << "]" << std::endl;
				}
			}

			output = net->predict(observation);
			assert(output.size() == 4 && "Drone expects 4 net outputs");
			drone->control(output[0], output[1], output[2], output[3]);
		}

		// ask for process
		return !someAlive;
	}

	void process() {
		/* std::cout << "EA Process Default" << std::endl; */
		auto elite = fitnessAgents();
		std::vector<Weights> eliteW;
		for (auto i : elite) {
			eliteW.push_back(populationW[i]);
		}

		/* auto selectedIds = tournamentSelection(); */
		auto selectedIds = sus(popSize);
		auto offspringWeights = crossover(selectedIds);
		mutation(offspringWeights);

		populationW = offspringWeights;

		for (int i = 0; i < eliteW.size(); ++i) {
			populationW[i] = eliteW[i];
		}

		generation += 1;
		resetAgents();
	}

	void process_without_crossover() {
		/* std::cout << "EA Process Without crossover" << std::endl; */
		auto elite = fitnessAgents();

		std::vector<Weights> eliteW;
		for (auto i : elite) {
			eliteW.push_back(populationW[i]);
		}

		const float factor = 0.2;
		auto selectedIds = sus(popSize*factor);
		auto offspringWeights = popUpscaling(selectedIds, std::ceil(1.0/factor));
		mutation(offspringWeights);

		populationW = offspringWeights;

		// keep first elite as the first in the pop
		populationW[0] = eliteW[0];
		for (int i = 1; i < eliteW.size(); ++i) {
			populationW[popSize*factor + i] = eliteW[i];
		}

		generation += 1;
		resetAgents();
	}

	void saveEA(const std::string &path) const {
		json popW;

		for (int i = 0; i < popSize; ++i) {
			popW[std::to_string(i)] = populationW[i];
		}

        json config = {
            {"popSize", popSize},
            {"motherNet", motherDescription},
			{"popW", popW}
        };

        std::ofstream file(path);
        file << config.dump(4);
        file.close();

		std::cout << "EA saved to a file " << path << std::endl;
	}

	void loadPopEA(const std::string &path) {
        std::ifstream input(path);
        json config;
        input >> config;
        input.close();

        size_t size = config["popSize"];
		assert(size == this->popSize && "Problem - Load size != EA pop size!");

		for (int i = 0; i < size; ++i) {
			populationW[i] = Weights(config["popW"][std::to_string(i)]);
		}

		resetAgents();
		std::cout << "EA popw finished loading from a file " << path << std::endl;
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

		float fitnessSum = 0;
		for (int i = 0; i < popSize; ++i) {
			fitness[i] += 1000 * agents[i]->goalIndex;
			fitnessSum += fitness[i];
		}

		const int eliteSize = popSize*0.05;
		std::vector<float> sortedFitness(eliteSize); //largest n numbers
		std::partial_sort_copy(
			std::begin(fitness), std::end(fitness), 
			std::begin(sortedFitness), std::end(sortedFitness), 
			std::greater() 
		);

		lastMaxFitness = sortedFitness[0];
		lastAverageFitness = fitnessSum / popSize;

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

	std::vector<size_t> sus(const int N) {
		std::vector<size_t> selectedIds;

		// A hope to have the most fit one as the first one (for rendering and stuff...)
		std::vector<float> selectedFs;  

		float fSum = 0;
		for (auto f : fitness) {
			fSum += f;
		}

		const float fDist = fSum/N;

		std::uniform_real_distribution<float> distr(0, fDist);
		const float startPoint = distr(gen);

		for (int n = 0; n < N; ++n) {

			float selectionPoint = startPoint + n*fDist;

			float testSum = 0;
			for (int i = 0; i < popSize; ++i) {
				testSum += fitness[i];

				if (selectionPoint <= testSum) {
					selectedIds.push_back(i);
					selectedFs.push_back(fitness[i]);
					break;
				}
			}
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

	std::vector<Weights> popUpscaling(const std::vector<size_t> &selectedIds, const size_t upscaleFactor) {
		std::vector<Weights> newPopW;
		newPopW.reserve(popSize);

		for (auto id : selectedIds) {
			newPopW.push_back(populationW[id]);
		}

		std::uniform_real_distribution<float> weightDistr(-0.1f, 0.1f);

		// for each of the selected ones
		for (int i = 0; i < selectedIds.size(); ++i) {
			// upscale them by factor - 1 (1 original + rest new)
			for (int _ = 0; _ < upscaleFactor-1; ++_) {
				Weights o1;
				o1.reserve(newPopW[i].size());
				// go through all of their weights and update them by a little
				for (int w = 0; w < newPopW[i].size(); ++w) {
					// WARN: let's say that we don't care about weights > 1 or < -1, we'll see how that goes
					o1.push_back(newPopW[i][w] + weightDistr(gen));
				}

				newPopW.push_back(o1);
			}
		}

		assert(newPopW.size() == popSize && "Pop after upscaling does not match the expected pop size");
		return newPopW;
	}

	void mutation(std::vector<Weights> &offspringW) {
		std::uniform_real_distribution<float> weightDistr(-1.0f, 1.0f);
		std::uniform_real_distribution<float> chanceDistr(0.0f, 1.0f);

		const float MUTPROB = 0.025;

		for (int i = 0; i < popSize; ++i) {
			for (int k = 0; k < offspringW[i].size(); ++k) {
				if (chanceDistr(gen) < MUTPROB) {
					offspringW[i][k] = weightDistr(gen);
				}
			}
		}
	}

	std::vector<Weights> populationW;
	std::vector<float> fitness;

	const size_t popSize;
	/* const Net mother; */
	const json motherDescription;
};
