#pragma once

#include <SFML/System/Vector2.hpp>
#include <cmath>
#include <math.h>
#include <iostream>

struct Thruster {
	float angleController;
	float powerController;

    float angle;

	const float maxAngle = M_PI * 0.5;
	const float maxPower = 15.0;

	Thruster() { reset(); }

	void reset() {
		angleController = 0;
		powerController = 0;
        angle = 0;
	}

    float getPower() {
        return powerController * maxPower;
    }

	void update(const float dt) {
        const float desiredAngle = angleController * maxAngle;
        /* const float desiredAngle = angleController; */
        const float speedOfTransition = 5.0f;

        angle += (desiredAngle - angle) * speedOfTransition * dt;
    }
};

struct Drone {
	sf::Vector2f pos;
    float angle;

	sf::Vector2f vel;
	float angularVel;

	Thruster thrusterLeft, thrusterRight;

	sf::Vector2f aabbSize;
	const sf::Vector2f startPos;
	const sf::Vector2f thrusterOffset{50,0};

	Drone(sf::Vector2f startPos) : startPos(startPos) { reset(); }

	void reset() {
		pos = startPos;
        angle = 0;

		vel = sf::Vector2f();
		angularVel = 0;
	}

    sf::Vector2f getThrust() {
        /* sf::Vector2f thrustLeft = */ 

        const float lAngle = angle + thrusterLeft.angle - M_PI * 0.5f;
        const float rAngle = angle + thrusterRight.angle - M_PI * 0.5f;

        sf::Vector2f lThrustVector{cos(lAngle), sin(lAngle)};
        sf::Vector2f rThrustVector{cos(rAngle), sin(rAngle)};

        return lThrustVector * thrusterLeft.getPower() + 
               rThrustVector * thrusterRight.getPower();
    }


    float getTorque() {
        const float lPower = thrusterLeft.getPower();
        const float rPower = thrusterRight.getPower();

        const float lAngle = thrusterLeft.angle - M_PI * 0.5f;
        const float rAngle = thrusterRight.angle - M_PI * 0.5f;
        sf::Vector2f lThrustVector{cos(lAngle), sin(lAngle)};
        sf::Vector2f rThrustVector{cos(rAngle), sin(rAngle)};

        // works for 2 symetrical thrusters
        float lTorque = cross(lPower*lThrustVector, -thrusterOffset);
        float rTorque = cross(rPower*rThrustVector,  thrusterOffset);

        // not really physicy but helps even out the cross product value
        const float momentOfInertia = 0.0005f;

        return (lTorque + rTorque)*momentOfInertia;
    }

	void update(const float dt) {
		thrusterLeft.update(dt);
		thrusterRight.update(dt);

		/* const sf::Vector2f gravity{0, 10}; */
		/* vel += gravity * dt; */

		/* vel += getThrust() * dt; */
        /* pos += vel; */

        angularVel += getTorque() * dt;
        /* angularVel = 0.01f; */
        angle += angularVel;
	}

private:
    float cross(sf::Vector2f a, sf::Vector2f b) {
        return a.x * b.y + a.y * b.x;
    }
};
