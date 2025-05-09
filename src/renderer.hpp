#pragma once

#include <SFML/Graphics.hpp>
#include <SFML/Graphics/CircleShape.hpp>
#include <SFML/Graphics/Color.hpp>
#include <SFML/Graphics/RectangleShape.hpp>
#include <SFML/Graphics/RenderWindow.hpp>
#include <SFML/System/Vector2.hpp>
#include <vector>

#include "drone.hpp"
#include "utils.hpp"

class Renderer {
    sf::RectangleShape base;
    sf::CircleShape center;
    sf::RectangleShape thrusterLeft;
    sf::RectangleShape thrusterLeftBottom;
    sf::RectangleShape thrusterRight;
    sf::RectangleShape thrusterRightBottom;

    sf::CircleShape collisionSphere;

    const sf::Color cBody = sf::Color(150,150,150);
    const sf::Color cCenter = sf::Color::Green;
    const sf::Color cCenterCollect = sf::Color(0,255,255);
    const sf::Color cThrusterOn = sf::Color::Magenta;
    const sf::Color cThrusterOff = sf::Color::White;

public:
    Renderer() {
        // Drone shapes
        base = sf::RectangleShape(sf::Vector2f{100, 20});
        base.setOrigin(base.getSize()/2.0f);
        base.setFillColor(cBody);

        center = sf::CircleShape(20);
        center.setOrigin(center.getRadius(),center.getRadius());
        center.setFillColor(cCenter);

        thrusterLeft = sf::RectangleShape(sf::Vector2f{15, 40});
        thrusterLeft.setOrigin(thrusterLeft.getSize()/2.0f);
        thrusterLeft.setFillColor(sf::Color::Red);

        thrusterRight = sf::RectangleShape(sf::Vector2f{15,40});
        thrusterRight.setOrigin(thrusterRight.getSize()/2.0f);
        thrusterRight.setFillColor(sf::Color::Blue);

        thrusterLeftBottom = sf::RectangleShape(sf::Vector2f{15,5});
        thrusterLeftBottom.setOrigin(sf::Vector2f{7.5, -10});
        thrusterLeftBottom.setFillColor(cThrusterOff);

        thrusterRightBottom = sf::RectangleShape(sf::Vector2f{15,5});
        thrusterRightBottom.setOrigin(sf::Vector2f{7.5, -10});
        thrusterRightBottom.setFillColor(cThrusterOff);
    }

	void draw_body(const Drone *drone,
				   sf::RenderWindow *target) {
		base.setPosition(drone->pos);
        center.setPosition(drone->pos);

        // vector rot
        /* x' = x cos θ − y sin θ */
        /* y' = x sin θ + y cos θ */
        const float cos_angle = cos(drone->angle);
        const float sin_angle = sin(drone->angle);
        const sf::Vector2f rotatedOffset{
            drone->thrusterOffset.x * cos_angle - drone->thrusterOffset.y * sin_angle,
            drone->thrusterOffset.x * sin_angle + drone->thrusterOffset.y * cos_angle,
        };

        thrusterLeft.setPosition(drone->pos  - rotatedOffset);
        thrusterRight.setPosition(drone->pos + rotatedOffset);
        thrusterLeftBottom.setPosition(drone->pos - rotatedOffset);
        thrusterRightBottom.setPosition(drone->pos + rotatedOffset);

        base.setRotation(RAD_TO_DEG * drone->angle);
        center.setRotation(RAD_TO_DEG * drone->angle);
        if (drone->goalTimer > 0) {
            center.setFillColor(cCenterCollect);
        }
        else {
            center.setFillColor(cCenter);
        }

        thrusterLeft.setRotation(RAD_TO_DEG * (drone->angle + drone->thrusterLeft.angle));
        thrusterRight.setRotation(RAD_TO_DEG * (drone->angle + drone->thrusterRight.angle));
        thrusterLeftBottom.setRotation(RAD_TO_DEG * (drone->angle + drone->thrusterLeft.angle));
        thrusterRightBottom.setRotation(RAD_TO_DEG * (drone->angle + drone->thrusterRight.angle));

        if (drone->thrusterRight.powerController > 0) {
            thrusterRightBottom.setFillColor(cThrusterOn);
        }
        else {
            thrusterRightBottom.setFillColor(cThrusterOff);
        }

        if (drone->thrusterLeft.powerController > 0) {
            thrusterLeftBottom.setFillColor(cThrusterOn);
        }
        else {
            thrusterLeftBottom.setFillColor(cThrusterOff);
        }

        target->draw(base);
        target->draw(center);
        target->draw(thrusterLeft);
        target->draw(thrusterRight);
        target->draw(thrusterLeftBottom);
        target->draw(thrusterRightBottom);
	}

	void draw_debug(const Drone *drone, 
                    const World &world, 
                    const std::vector<std::unique_ptr<Drone>> &drones,
                    sf::RenderWindow *target) {

        collisionSphere.setRadius(drone->contactRadius);
        collisionSphere.setOrigin(collisionSphere.getRadius(), collisionSphere.getRadius());
        collisionSphere.setPointCount(16);
        collisionSphere.setFillColor(sf::Color::Transparent);
        collisionSphere.setOutlineThickness(2);
        collisionSphere.setOutlineColor(sf::Color(0,100,0));

        collisionSphere.setPosition(drone->pos);

        target->draw(collisionSphere);

        for (auto && s : drone->sensors) {

            sf::Vector2f dir = s.getDir(drone);
            sf::Vector2f start = drone->pos + dir*drone->contactRadius;

            float check = s.check(drone, world, drones);

            // free sensor without 
            sf::Vector2f endMax = drone->pos + dir*(drone->contactRadius + s.length);
            sf::Vertex lineMax [] = {{{start.x,  start.y}, sf::Color::White}, 
                                     {{endMax.x, endMax.y}, sf::Color::White}};
            target->draw(lineMax, 2, sf::Lines);

            // hit
            if (check < 1.0) {
                sf::Vector2f endCurr = drone->pos + dir*(drone->contactRadius + check*s.length);
                sf::Vertex lineCurr[] = {{{start.x,   start.y}, sf::Color::Red}, 
                                         {{endCurr.x, endCurr.y}, sf::Color::Red}};
                target->draw(lineCurr, 2, sf::Lines);
            }
        }
    }
};
