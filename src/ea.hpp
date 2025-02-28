#pragma once

#include "drone.hpp"
#include "net.hpp"
#include <SFML/Graphics/BlendMode.hpp>
#include <SFML/System/Vector2.hpp>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <iterator>
#include <memory>
#include <random>
#include <vector>

using Individual = std::unique_ptr<Net>;
using Agent = std::unique_ptr<Drone>;

constexpr float HALF_PI = M_PI * 0.5f;

struct EA {
	std::vector<sf::Vector2f> goals;
	std::vector<Agent> agents;
	bool simFinished = false;

	EA(size_t popSize, const Net &mother, const Drone &father)
		: popSize(popSize) {
		assert(popSize % 2 == 0 && "PopSize should be divisible by 2!");
		initPop(mother);
		initAgents(father);
		simFinished = false;

		goals = {sf::Vector2f{200, 200}, sf::Vector2f{200, 600},
				 sf::Vector2f{600, 600}, sf::Vector2f{600, 200},
				 sf::Vector2f{400, 400}};
	}

	void update(const float dt, const sf::Vector2f &boundary) {
		sf::Vector2f goalDist;
		std::vector<float> observation;
		Output output;

		bool someAlive = false;

		for (int i = 0; i < popSize; ++i) {
			auto drone = agents[i].get();
			auto net = population[i].get();

			drone->update(dt, boundary);

			if (!drone->alive) continue;
			someAlive = true;

			goalDist = goals[drone->goalIndex] - drone->pos;
			observation = {
				drone->pos.x / 800,
				drone->pos.y / 800,
				drone->vel.x / 20.0f,
				drone->vel.y / 20.0f,
				(drone->angle) / HALF_PI,
				(drone->thrusterLeft.angle) / HALF_PI,
				(drone->thrusterRight.angle) / HALF_PI,
				goalDist.x / 800,
				goalDist.y / 800
			};

			output = net->predict(observation);
			drone->control(output[0], output[1], output[2], output[3]);
		}

		if (!someAlive) {
			simFinished = true;
		}
	}

	void process() {
		size_t bestID = fitnessAgents();
		std::cout << bestID << std::endl;
		Weights elite = populationW[bestID];

		std::cout << "EA PROCESS" << std::endl;
		auto selectedIds = tournamentSelection();
		std::cout << "Tournament done" << std::endl;
		auto offspringWeights = crossover(selectedIds);
		std::cout << "Crossover done" << std::endl;
		mutation(offspringWeights);
		std::cout << "Mutation done" << std::endl;

		populationW = offspringWeights;
		populationW[0] = elite;

		resetAgents();
		std::cout << "Agents reseted" << std::endl;
		simFinished = false;
	}

  private:
	void initPop(const Net &mother) {
		populationW.resize(popSize);

		for (int i = 0; i < popSize; ++i) {
			population.push_back(std::make_unique<Net>());

			for (const auto & mod : mother.modules) {
				population[i]->modules.push_back(mod->clone());
			}

			population[i]->initialize();
			populationW[i] = population[i]->getWeights();
		}
	}

	size_t fitnessAgents() {
		for (int i = 0; i < popSize; ++i) {
			fitness[i] = agents[i]->aliveCounter + 500 * agents[i]->goalIndex;
			std::cout << fitness[i] << std::endl;
		}

		auto max = std::max_element(fitness.begin(), fitness.end());
		int argmax = std::distance(fitness.begin(), max);
		std::cout << argmax << std::endl;
		std::cout << "MaxFitness: " << *max << std::endl;
		return argmax;
	}

	void initAgents(const Drone &father) {
		for (int i = 0; i < popSize; ++i) {
			agents.push_back(std::make_unique<Drone>(father.startPos));
		}

		fitness.resize(popSize);
	}

	void resetAgents() {
		for (int i = 0; i < popSize; ++i) {
			agents[i]->reset();
			population[i]->loadWeights(populationW[i]);
		}
	}

	std::vector<size_t> tournamentSelection() {
		std::vector<size_t> selectedIds{popSize};
		std::uniform_int_distribution<std::mt19937::result_type> distr(
			0, popSize - 1);

		const int tournamentSize = 2;
		std::vector<float> tournamentFs{tournamentSize};
		std::vector<size_t> tournamentIds{tournamentSize};

		std::cout << "Selected ids" << std::endl;
		for (int i = 0; i < popSize; ++i) {
			// running the tournament
			for (int x = 0; x < tournamentSize; ++x) {
				auto id = distr(gen);
				tournamentFs[x] = fitness[id];
				tournamentIds[x] = id;
			}

			auto maxFitness =
				std::max_element(tournamentFs.begin(), tournamentFs.end());
			int argmax = std::distance(tournamentFs.begin(), maxFitness);

			// selecte best one from current tournament
			selectedIds[i] = tournamentIds[argmax];
		}

		return selectedIds;
	}

	std::vector<Weights> crossover(const std::vector<size_t> &selectedIds) {
		std::uniform_real_distribution<float> distr(0.0f, 1.0f);

		std::vector<Weights> newPopW(popSize);

		for (int i = 0; i < popSize; i += 2) {
			auto p1 = populationW[selectedIds[i]];
			auto p2 = populationW[selectedIds[i + 1]];

			Weights o1(p1.size());
			Weights o2(p2.size());

			for (int k = 0; k < p1.size(); ++k) {
				if (distr(gen) < 0.5) {
					o1[k] = p1[k];
					o2[k] = p2[k];
				} else {
					o1[k] = p2[k];
					o2[k] = p1[k];
				}
			}

			newPopW[i] = o1;
			newPopW[i + 1] = o2;
		}

		return newPopW;
	}

	void mutation(std::vector<Weights> &offspringW) {
		std::uniform_real_distribution<float> weightDistr(-1.0f, 1.0f);
		std::uniform_real_distribution<float> chanceDistr(0.0f, 1.0f);

		const float MUTPROB = 1.0;

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
};
