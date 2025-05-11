#pragma once

#include <SFML/System/Vector2.hpp>
#include <cstdint>
#include <random>
#include <vector>

inline std::mt19937 gen(time(nullptr));

constexpr float HALF_PI = M_PI * 0.5f;

constexpr double RAD_TO_DEG = 57.295779513082320876798154814105;
constexpr double DEG_TO_RAD = M_PI / 180.0f;

const uint32_t winHeight = 800;
const uint32_t winWidth = 800;

const sf::Vector2f droneStart{400,650};

constexpr float dt = 1.f / 60.f;

inline float dist(const sf::Vector2f &vec) {
	return sqrt((vec.x*vec.x)+(vec.y*vec.y));
}

struct Wall{
	sf::Vector2f pos;
	float radius;
};

struct World {
	sf::Vector2f boundary;
	std::vector<Wall> walls;
	std::vector<sf::Vector2f> goals;
	bool isStatic = true;

	void randomize() {
		if (isStatic) return;

		std::uniform_int_distribution<std::mt19937::result_type> goalPosDistr(25, winWidth-25);

		std::uniform_int_distribution<std::mt19937::result_type> posDistr(0, winWidth);
		std::uniform_int_distribution<std::mt19937::result_type> sizeDistr(50, winWidth/8);

		for (int i = 0; i < walls.size(); ++i) {
			while (true) {
				walls[i] = Wall{sf::Vector2f{(float)posDistr(gen), (float)posDistr(gen)}, (float)sizeDistr(gen)};
				if (dist(droneStart - walls[i].pos) - walls[i].radius >= 60) {
					break;
				}
			}
		}

		for (int i = 0; i < goals.size(); ++i) {
			while (true) {
				goals[i] = sf::Vector2f{(float)goalPosDistr(gen), (float)goalPosDistr(gen)};

				bool foundProblem = false;
				for (int w = 0; w < walls.size(); ++w) {
					// check if walls are too close to the goal (at least 60 around the walls) 
					if (dist(goals[i] - walls[w].pos) - walls[w].radius < 60) {
						foundProblem = true;
						break;
					}
				}

				if (!foundProblem) break;
			}
		}
	}
};

