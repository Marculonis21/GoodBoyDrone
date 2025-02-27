#pragma once

#include "drone.hpp"
#include "net.hpp"
#include <SFML/Graphics/BlendMode.hpp>
#include <SFML/System/Vector2.hpp>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <memory>
#include <random>
#include <tuple>
#include <vector>
#include <iostream>

using Individual = std::unique_ptr<Net>;
using Agent = std::unique_ptr<Drone>;

constexpr float HALF_PI = M_PI*0.5f;

struct EA {
    std::vector<sf::Vector2f> goals;
    std::vector<Agent> agents;
    bool simFinished = false;

    EA(size_t popSize, const Net &mother, const Drone &father) : popSize(popSize) { 
        initPop(mother);
        initAgents(father);
        simFinished = false;

        goals = {
            sf::Vector2f{200,200},
            sf::Vector2f{200,600},
            sf::Vector2f{600,600},
            sf::Vector2f{600,200},
            sf::Vector2f{400,400}
        };
    }
    
    void update(const float dt, const sf::Vector2f &boundary) {
        sf::Vector2f goalDist;
        std::vector<float> observation;
        Output output;

        Drone* drone;
        Net* net;

        bool someAlive = false;

        for (int i = 0; i < popSize; ++i) {
            drone = agents[i].get();
            net = population[i].get();

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

        resetAgents();
        simFinished = false;
    }

private:
    void initPop(const Net &mother) {
        for (int i = 0; i < popSize; ++i) {
            population.push_back(std::make_unique<Net>());
            population[i]->modules = mother.modules;
            population[i]->initialize();
        }
    }

    std::vector<float> fitnessAgents() {
        for (int i = 0; i < popSize; ++i) {
            fitness[i] = agents[i]->aliveCounter + 250*agents[i]->goalIndex;
        }

        auto max = std::max_element(fitness.begin(), fitness.end());
        int argmax = std::distance(fitness.begin(), max);
        float maxValue = fitness[argmax];
        std::cout << "MaxFitness: " << maxValue << std::endl;

        return fitness;
    }

    void initAgents(const Drone &father) {
        for (int i = 0; i < popSize; ++i) {
            agents.push_back(std::make_unique<Drone>(father.startPos));
        }

        fitness.resize(popSize);
    }

    void resetAgents() {
        for (auto && drone : agents) {
            drone.reset();
        }
    }

    std::vector<size_t> tournamentSelection() {
        std::vector<size_t> selected_ids;
        std::uniform_int_distribution<std::mt19937::result_type> distr(0, popSize);


        const int tournamentSize = 2;
        std::vector<float> tournamentFs;
        std::vector<size_t> tournamentIds;
        tournamentFs.resize(tournamentSize);
        tournamentIds.resize(tournamentSize);

        for (int i = 0; i < popSize; ++i) {
            for (int x = 0; x < tournamentSize; ++x) {
                auto id = distr(gen);
                tournamentFs[x] = fitness[id];
                tournamentIds[x] = id;
            }

        }
    }

    std::vector<Individual> population;
    std::vector<float> fitness;

    const size_t popSize;
};
