#pragma once

#include "drone.hpp"
#include "net.hpp"
#include <memory>
#include <vector>

using Individual = std::unique_ptr<Net>;
using Agent = std::unique_ptr<Drone>;

using SynapsePopulation = std::vector<float>;
using MetaPopulation = std::vector<SynapsePopulation>;

// https://jmlr.csail.mit.edu/papers/volume9/gomez08a/gomez08a.pdf
struct CoSyNE {
	float lastMaxFitness = 0;
	float lastAverageFitness = 0;

	uint64_t generation = 0;

	size_t input_size;

	CoSyNE(size_t individualCount, const Net &mother, const Drone &father) : individualCount(individualCount), synapseCount(mother.getWeights().size()), motherDescription(mother.describe()) {

		population.reserve(individualCount);
		populationW.reserve(individualCount);
		agents.reserve(individualCount);
		fitness.resize(individualCount);

		initPop(mother);
		initAgents(father);

		input_size = mother.input_size;
	}

private:
	void initPop(const Net &mother) {
		for (int i = 0; i < individualCount; ++i) {
			population.push_back(std::make_unique<Net>());

			for (const auto & mod : mother.modules) {
				population[i]->modules.push_back(mod->clone());
			}

			population[i]->initialize();
			populationW.push_back(population[i]->getWeights());
		}

		metaPopulation.reserve(synapseCount);

		synapsePopulation.resize(individualCount);
		for (int s = 0; s < synapseCount; ++s) {
			for (int ind = 0; ind < individualCount; ++ind) {
				synapsePopulation[ind] = populationW[ind][s];
			}

			metaPopulation.push_back(synapsePopulation);
		}
	}

	void initAgents(const Drone &father) {
		for (int i = 0; i < individualCount; ++i) {
			agents.push_back(std::make_unique<Drone>(father.startPos));
		}
	}

	/* void resetAgents() { */
	/* 	for (int i = 0; i < popSize; ++i) { */
	/* 		agents[i]->reset(); */
	/* 		population[i]->loadWeights(populationW[i]); */
	/* 		fitness[i] = 0; */
	/* 	} */
	/* } */


	SynapsePopulation synapsePopulation;
	MetaPopulation metaPopulation;

	std::vector<Agent> agents;
	std::vector<Individual> population;
	std::vector<Weights> populationW;
	std::vector<float> fitness;

	const size_t individualCount;
	const size_t synapseCount;
	const json motherDescription;
};

