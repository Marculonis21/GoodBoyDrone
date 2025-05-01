#pragma once

#include "drone.hpp"
#include "net.hpp"
#include <cassert>
#include <memory>
#include <random>
#include <vector>

using Individual = std::unique_ptr<Net>;
using Agent = std::unique_ptr<Drone>;

using SynapsePopulation = std::vector<float>;
using MetaPopulation = std::vector<SynapsePopulation>;

struct EAItem {
	Net *net; 
	Drone *drone; 
};

// https://jmlr.csail.mit.edu/papers/volume9/gomez08a/gomez08a.pdf
struct EA {
	float lastMaxFitness = 0;
	float lastAverageFitness = 0;

	uint64_t generation = 0;

	size_t input_size;

	EA(size_t popSize, const Net &mother, const Drone &father) : popSize(popSize), synapseCount(mother.getWeights().size()), motherDescription(mother.describe()) {

		population.reserve(popSize);
		populationW.reserve(popSize);
		agents.reserve(popSize);
		fitness.resize(popSize);

		initPop(mother);
		initAgents(father);

		input_size = mother.input_size;
	}

	const EAItem operator [](int idx) const {
		return EAItem{population[idx].get(), agents[idx].get()};
	}

	bool update(const float dt, const World &world, bool debug=false) {
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

			/* drone->genObservation_with_sensors(observation, world); */
			drone->genObservation_no_sensors(observation, world);

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

	void process() {
		std::cout << "COSYNE Process" << std::endl;

		std::vector<size_t> fitnessOrder = fitnessAgents();

		const size_t parentCount = popSize * 0.25;
		/* std::cout << "PC" << parentCount << std::endl; */

		auto offspringPopW = crossover(fitnessOrder, parentCount);
		mutation(offspringPopW);

		std::vector<Weights> newPopulationW(popSize);

		// Transfer the top X parents over to the new pop
		for (int i = 0; i < parentCount; ++i) {
			newPopulationW[i] = populationW[fitnessOrder[i]];
		}

		// Fill the rest with new offsprings
		for (int i = parentCount; i < popSize; ++i) {
			newPopulationW[i] = offspringPopW[i-parentCount];
		}

		convert_WeightsToMeta(newPopulationW);

		permuteMeta(fitnessOrder);

		convert_MetaToWeights();

		generation += 1;
		resetAgents();
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

		metaPopulation.reserve(synapseCount);

		for (int s = 0; s < synapseCount; ++s) {
			metaPopulation.push_back(SynapsePopulation(popSize));
		}

		convert_WeightsToMeta(populationW);
	}

	void initAgents(const Drone &father) {
		for (int i = 0; i < popSize; ++i) {
			agents.push_back(std::make_unique<Drone>(father.startPos));
		}
	}

	void convert_WeightsToMeta(const std::vector<Weights> &popW) {
		for (int s = 0; s < synapseCount; ++s) {
			for (int ind = 0; ind < popSize; ++ind) {
				metaPopulation[s][ind] = popW[ind][s];
			}
		}
	}

	void convert_MetaToWeights() {
		for (int s = 0; s < synapseCount; ++s) {
			for (int ind = 0; ind < popSize; ++ind) {
				populationW[ind][s] = metaPopulation[s][ind];
			}
		}
	}

	void resetAgents() {
		for (int i = 0; i < popSize; ++i) {
			agents[i]->reset();
			population[i]->loadWeights(populationW[i]);
			fitness[i] = 0;
		}
	}

	// get indices corresponding to the sorted fitness values (without sorting them)
	std::vector<size_t> fitnessAgents() {
		// produce the final fitness value for each agent
		float fitnessSum = 0;
		for (int i = 0; i < popSize; ++i) {
			fitness[i] += 1000 * agents[i]->goalIndex;
			fitnessSum += fitness[i];
		}

		std::vector<size_t> idx(fitness.size());
		std::iota(idx.begin(), idx.end(), 0);
		std::sort(idx.begin(), idx.end(), [&](size_t a, size_t b){return fitness[a] > fitness[b];});

		assert(fitness[idx[0]] >= fitness[idx[1]] && "Fitness sorting order incorrect");

		lastMaxFitness = fitness[idx[0]];
		lastAverageFitness = fitnessSum / popSize;

		return idx;
	}

	std::vector<Weights> crossover(const std::vector<size_t> &fitnessOrder, const size_t parentCount) {
		assert((popSize-parentCount)%2 == 0 && "It would be nice if this worked out");

		std::uniform_real_distribution<float> crossoverDistr(0.0f, 1.0f);
		std::uniform_int_distribution<std::mt19937::result_type> parentDistr(0, parentCount-1);

		std::vector<Weights> offspringPopW;
		offspringPopW.reserve(popSize-parentCount); // generate the rest of the population alongside parents

		for (int i = 0; i < popSize-parentCount; i += 2) {
			Weights p1 = populationW[fitnessOrder[parentDistr(gen)]];
			Weights p2 = populationW[fitnessOrder[parentDistr(gen)]];

			Weights o1;
			Weights o2;
			o1.reserve(p1.size());
			o2.reserve(p2.size());

			for (int k = 0; k < p1.size(); ++k) {
				if (crossoverDistr(gen) < 0.5) {
					o1.push_back(p1[k]);
					o2.push_back(p2[k]);
				} else {
					o1.push_back(p2[k]);
					o2.push_back(p1[k]);
				}
			}

			offspringPopW.push_back(o1);
			offspringPopW.push_back(o2);
		}

		return offspringPopW;
	}

	void mutation(std::vector<Weights> &offspringPopW) {
		const float MUTPROB = 0.025f;
		const float PERTURBATION = 0.1f;

		/* std::cauchy_distribution<float> chanceDistr(0, 0.3); */
		std::uniform_real_distribution<float> chanceDistr(0.0, 1.0);
		std::uniform_real_distribution<float> weightDistr(-PERTURBATION, PERTURBATION);

		for (int i = 0; i < offspringPopW.size(); ++i) {
			for (int k = 0; k < offspringPopW[i].size(); ++k) {
				// WARN: let's say that we don't care about weights > 1 or < -1, we'll see how that goes
				offspringPopW[i][k] += ((chanceDistr(gen) < MUTPROB) ? weightDistr(gen) : 0.0f);
			}
		}
	}

	std::vector<float> _precompute_markProbability(const std::vector<size_t> &fitnessOrder) {
		const float MAXFIT = fitness[fitnessOrder[0]];
		const float MINFIT = fitness[fitnessOrder[popSize-1]];

		std::vector<float> markProbability;
		markProbability.reserve(popSize);

		for (int i = 0; i < popSize; ++i) {
			markProbability.push_back(1.0 - std::pow((fitness[fitnessOrder[i]] - MINFIT)/(MAXFIT - MINFIT), 1.0/2.0));
		}

		return markProbability;
	}

	void _permuteMarkedMeta(const std::vector<size_t> &marked, const size_t synIndex) {
		// random cycle
		std::vector<size_t> perm(marked.size());
		std::iota(perm.begin(), perm.end(), 0);

		for (int i = 0; i < marked.size()-1; ++i) {
			std::uniform_int_distribution<std::mt19937::result_type> dis(i+1, marked.size()-1);
			int j = dis(gen);
			std::swap(perm[i], perm[j]);
		}

		SynapsePopulation columnCopy{metaPopulation[synIndex]};

		// apply the permutation to the correct population
		for (int i = 0; i < marked.size(); ++i) {
			metaPopulation[synIndex][marked[i]] = columnCopy[marked[perm[i]]];
		}
	}

	void permuteMeta(const std::vector<size_t> &fitnessOrder) {
		std::uniform_real_distribution<float> markDistr(0.0f, 1.0f);

		std::vector<float> markProbability = _precompute_markProbability(fitnessOrder);

		std::vector<size_t> marked;
		
		// over the sub-populations
		for (size_t s = 0; s < synapseCount; ++s) { 
			marked.clear();

			for (size_t i = 0; i < popSize; ++i) {
				if (markDistr(gen) < markProbability[i]) {
					marked.push_back(i);
				}
			}

			/* std::cout << s<<": MARKED: " << marked.size() << std::endl; */
			if (marked.size() > 1) {
				_permuteMarkedMeta(marked, s);
			}
		}
	}

	MetaPopulation metaPopulation;

	std::vector<Agent> agents;
	std::vector<Individual> population;
	std::vector<Weights> populationW;
	std::vector<float> fitness;

	const size_t popSize;
	const size_t synapseCount;
	const json motherDescription;
};

