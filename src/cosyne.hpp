#pragma once

#include "ea.hpp"

using Individual = std::unique_ptr<Net>;
using Agent = std::unique_ptr<Drone>;

using SynapsePopulation = std::vector<float>;
using MetaPopulation = std::vector<SynapsePopulation>;

// https://jmlr.csail.mit.edu/papers/volume9/gomez08a/gomez08a.pdf
struct CoSyNE : public AbstractEA {
	CoSyNE(size_t popSize, const Net &mother, const Drone &father) : AbstractEA(popSize, mother, father), synapseCount(mother.getWeights().size()) { 
		initPop(mother);
		initAgents(father);
	}

	void process() override {
		std::cout << "COSYNE Process" << std::endl;

		std::vector<size_t> fitnessOrder = fitnessAgents();

		const size_t parentCount = popSize * 0.25;

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

	void saveProcedure(const std::string &path) const override {
		json popW;

		for (int i = 0; i < popSize; ++i) {
			popW[std::to_string(i)] = populationW[i];
		}

        json config = {
			{"type", "CoSyNE"},
            {"popSize", popSize},
			{"synapseCount", synapseCount},
            {"motherNet", motherDescription},
			{"popW", popW}
        };

        std::ofstream file(path);
        file << config.dump(4);
        file.close();

		std::cout << "CoSyNE saved to a file: " << path << std::endl;
	}

private:
	MetaPopulation metaPopulation;
	const size_t synapseCount;

	void initPop(const Net &mother) override {
		AbstractEA::initPop(mother);

		metaPopulation.reserve(synapseCount);
		for (int s = 0; s < synapseCount; ++s) {
			metaPopulation.push_back(SynapsePopulation(popSize));
		}

		convert_WeightsToMeta(populationW);
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

	// get indices corresponding to the sorted fitness values (without sorting them)
	std::vector<size_t> fitnessAgents() override {
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

		lastFitnessStats.max = fitness[idx[0]];
		lastFitnessStats.min = fitness[idx[popSize-1]];
		lastFitnessStats.med = fitness[idx[popSize/2]];
		lastFitnessStats.avg = fitnessSum / popSize;;

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
		const float MUTPROB = 0.05f;

		std::uniform_real_distribution<float> chanceDistr(0.0, 1.0);
		std::cauchy_distribution<float> perturbedDistr(0, 0.3);

		for (int i = 0; i < offspringPopW.size(); ++i) {
			for (int k = 0; k < offspringPopW[i].size(); ++k) {
				// WARN: let's say that we don't care about weights > 1 or < -1, we'll see how that goes
				offspringPopW[i][k] += ((chanceDistr(gen) < MUTPROB) ? perturbedDistr(gen) : 0.0f);
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

			if (marked.size() > 1) {
				_permuteMarkedMeta(marked, s);
			}
		}
	}
};

