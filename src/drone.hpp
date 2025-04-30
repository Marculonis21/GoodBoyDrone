#pragma once

#include "utils.hpp"
#include <SFML/Graphics/CircleShape.hpp>
#include <SFML/System/Vector2.hpp>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <math.h>
#include <vector>
#include <memory>
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
        const float speedOfTransition = 5.0f;

        angle += (desiredAngle - angle) * speedOfTransition * dt;
    }

    void control(float ac, float pc) {
        this->angleController = ac;
        this->powerController = pc;
    }
};

struct Drone {

    struct Sensor {
        const float angle; 
        const float length; 

        Sensor(float angle, float length) : angle(angle), length(length) { }

        float check(const Drone* from, 
                    const World &world, 
                    const std::vector<std::unique_ptr<Drone>> &drones) const {
            sf::Vector2f dir{cos(angle+from->angle), sin(angle+from->angle)};

            // ray march - here we goooo!
            sf::Vector2f test = from->pos + dir*from->contactRadius;

            const std::vector<sf::Vector2f> worldWalls {
                sf::Vector2f{1,0},
                sf::Vector2f{0,1},
                sf::Vector2f{-1,0},
                sf::Vector2f{0,-1}
            };

            float checked = 0;
            while (checked < length) {
                float closest = std::numeric_limits<float>::max();
                float check = 0;
                for (auto && w : world.walls) {
                    check = dist(w.pos - test) - w.radius;
                    if (check < closest) {
                        closest = check;
                    }
                }

                // make the outer edge a wall too

                for (int i = 0; i < worldWalls.size(); ++i) {
                    check = dist(sf::Vector2f{test.x*worldWalls[i].x, test.y*worldWalls[i].y});
                    if (i > 1) { check = world.boundary.x - check; }

                    if (check < closest) {
                        closest = check;
                    }
                }

                // inside an object OR really close
                if (closest < 1) {
                    return checked/length;
                }

                checked += closest;
                test += dir*closest;
            }

            return 1.0;
        }

    private: 
        float dist(const sf::Vector2f &vec) const {
            return sqrt((vec.x*vec.x)+(vec.y*vec.y));
        }
    };

	sf::Vector2f pos;
    float angle;

	sf::Vector2f vel;
	float angularVel;

	Thruster thrusterLeft, thrusterRight;
    std::vector<Sensor> sensors = {
        Sensor{M_PI*0.00, 200},
        Sensor{M_PI*0.25, 200},
        Sensor{M_PI*0.50, 200},
        Sensor{M_PI*0.75, 200},
        Sensor{M_PI*1.00, 200},
        Sensor{M_PI*1.25, 200},
        Sensor{M_PI*1.50, 200},
        Sensor{M_PI*1.75, 200}
    };

	const float contactRadius = 60;
	const sf::Vector2f startPos;
	const sf::Vector2f thrusterOffset{50,0};

    uint64_t aliveTimer = 0;
    bool alive = true;

    size_t goalIndex = 0;
    size_t goalTimer = 0;

    std::vector<float> lastControls;

	Drone(sf::Vector2f startPos) : startPos(startPos) { 
        lastControls.resize(4);
        reset(); 
    }

	void reset() {
		pos = startPos;
        angle = 0;

		vel = sf::Vector2f();
		angularVel = 0;

        aliveTimer = 0;
        alive = true;

        goalIndex = 0;
        goalTimer = 0;

        thrusterLeft.reset();
        thrusterRight.reset();

        lastControls[0] = 0;
        lastControls[1] = 0;
        lastControls[2] = 0;
        lastControls[3] = 0;
	}

    sf::Vector2f getThrust() {
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

    void update(const float dt, const World &world) {
        if (!alive) return;
        aliveTimer += 1;

		thrusterLeft.update(dt);
		thrusterRight.update(dt);

		const sf::Vector2f gravity{0, 10};
		vel += gravity * dt;

		vel += getThrust() * dt;
        pos += vel;

        angularVel += getTorque() * dt;
        angle += angularVel;

        alive = !wait_he_should_be_already_dead(world);

        for (auto && w : world.walls) {
            auto v = w.pos - this->pos;
            float dist = (v.x*v.x)+(v.y*v.y);
            float minDist = w.radius + this->contactRadius;
            if (dist < minDist*minDist) {
                alive = false;
                return;
            }
        }
	}

    void genObservation_with_sensors(std::vector<float> &observation, const World &world) {
        assert(observation.size() == 15 && "genObservation_with_sensors wants to generate 15 obs");

        sf::Vector2f goalDist = world.goals[goalIndex % world.goals.size()] - pos;

        // creating observations 
        observation[0] = vel.x / 20.0f;
        observation[1] = vel.y / 20.0f;
        observation[2] = angularVel;
        observation[3] = cos(angle);
        observation[4] = sin(angle);
        observation[5] = goalDist.x / world.boundary.x;
        observation[6] = goalDist.y / world.boundary.y;

        for (int s = 0; s < sensors.size(); ++s) {
            observation[7+s] = 1 - sensors[s].check(this, world, {});
        }

        /* for (int l = 0; l < lastControls.size(); ++l) { */
        /*     observation[13+l] = lastControls[l]; */
        /* } */
    }

    void genObservation_no_sensors(std::vector<float> &observation, const World &world) {
        assert(observation.size() == 7 && "genObservation_no_sensors wants to generate 7 obs");

        sf::Vector2f goalDist = world.goals[goalIndex % world.goals.size()] - pos;

        // creating observations 
        observation[0] = vel.x / 20.0f;
        observation[1] = vel.y / 20.0f;
        observation[2] = angularVel;
        observation[3] = cos(angle);
        observation[4] = sin(angle);
        observation[5] = goalDist.x / world.boundary.x;
        observation[6] = goalDist.y / world.boundary.y;
    }

    void control(float lac, float lpc, float rac, float rpc) {
        lastControls[0] = lac;
        lastControls[1] = lpc;
        lastControls[2] = rac;
        lastControls[3] = rpc;

        lpc = (lpc+1) * 0.5;
        rpc = (rpc+1) * 0.5;

        thrusterLeft.control(lac, lpc);
        thrusterRight.control(rac, rpc);
    }

private:
    float cross(const sf::Vector2f &a, const sf::Vector2f &b) {
        return a.x * b.y + a.y * b.x;
    }

    bool wait_he_should_be_already_dead(const World &world) {
        return pos.x < 0 || pos.x > world.boundary.x ||
               pos.y < 0 || pos.y > world.boundary.y ||
               angle < -M_PI * 0.5f || 
               angle > +M_PI * 0.5f ||
               aliveTimer > 600 ||
               goalIndex > 2*world.goals.size();
    }
};
