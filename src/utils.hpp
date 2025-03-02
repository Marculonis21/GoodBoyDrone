#pragma once

#include <SFML/System/Vector2.hpp>
#include <random>

inline std::mt19937 gen(time(nullptr));

constexpr float HALF_PI = M_PI * 0.5f;

constexpr double RAD_TO_DEG = 57.295779513082320876798154814105;

const uint32_t winHeight = 800;
const uint32_t winWidth = 800;
const sf::Vector2f boundary{winWidth, winHeight};

struct Wall{
	sf::Vector2f pos;
	float radius;
};

