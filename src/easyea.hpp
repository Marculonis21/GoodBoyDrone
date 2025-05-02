#pragma once

#include "ea.hpp"

struct EasyEA : public AbstractEA {

	EasyEA(size_t popSize, const Net &mother, const Drone &father) : AbstractEA(popSize, mother, father) { }

	// PROCESS WITHOUT CROSSOVER
	void process() override {
		/* std::cout << "EasyEA Process Without crossover" << std::endl; */
		auto elite = fitnessAgents();

		std::vector<Weights> eliteW;
		for (auto i : elite) {
			eliteW.push_back(populationW[i]);
		}

		const float factor = 0.2;
		auto selectedIds = top_n(popSize*factor);
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

	void saveProcedure(const std::string &path) const override {
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

	void loadProcedure(const std::string &path) override {
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
	// return an elite vector
	std::vector<size_t> fitnessAgents() override {
		std::vector<size_t> eliteIds;

		float fitnessSum = 0;
		for (int i = 0; i < popSize; ++i) {
			fitness[i] += 1000 * agents[i]->goalIndex;
			fitnessSum += fitness[i];
		}

		const int eliteSize = popSize*0.05;

		std::vector<size_t> idx(popSize);
		std::iota(idx.begin(), idx.end(), 0);
		std::sort(idx.begin(), idx.end(), [&](size_t a, size_t b){return fitness[a] > fitness[b];});
		assert(fitness[idx[0]] >= fitness[idx[1]] && "Fitness sorting order incorrect");

		for (int i = 0; i < eliteSize; ++i) {
			eliteIds.push_back(idx[i]);
		}

		lastFitnessStats.max = fitness[idx[0]];
		lastFitnessStats.min = fitness[idx[popSize-1]];
		lastFitnessStats.med = fitness[idx[popSize/2]];
		lastFitnessStats.avg = fitnessSum / popSize;;

		return eliteIds;
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
					break;
				}
			}
		}

		return selectedIds;
	}

	std::vector<size_t> top_n(const int N) {
		std::vector<size_t> selectedIds;

		std::vector<size_t> indices(fitness.size());

		std::iota(indices.begin(), indices.end(), 0);
		std::sort(indices.begin(), indices.end(),
				[&](size_t a, size_t b) -> bool {
					return fitness[a] > fitness[b];
				});
	
		for (int i = 0; i < N; ++i) {
			selectedIds.push_back(indices[i]);
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
		std::uniform_real_distribution<float> chanceDistr(0.0f, 1.0f);
		const float wChangeProb = 0.25f;

		// for each of the selected ones
		for (int i = 0; i < selectedIds.size(); ++i) {
			// upscale them by factor - 1 (1 original + rest new)
			for (int _ = 0; _ < upscaleFactor-1; ++_) {
				Weights o1;
				o1.reserve(newPopW[i].size());
				// go through all of their weights and update them by a little
				for (int w = 0; w < newPopW[i].size(); ++w) {
					// WARN: let's say that we don't care about weights > 1 or < -1, we'll see how that goes
					
					// prob chance that a specific w is going to be changed (otherwise just coppied)
					o1.push_back(newPopW[i][w] + ((chanceDistr(gen) < wChangeProb) ? weightDistr(gen) : 0.0f));
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
};
