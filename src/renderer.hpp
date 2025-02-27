#pragma once

#include <SFML/Graphics.hpp>
#include <SFML/Graphics/CircleShape.hpp>
#include <SFML/Graphics/Color.hpp>
#include <SFML/Graphics/RectangleShape.hpp>
#include <SFML/System/Vector2.hpp>
#include "drone.hpp"

constexpr double RAD_TO_DEG = 57.295779513082320876798154814105;

class Renderer {
    sf::RectangleShape base;
    sf::CircleShape center;
    sf::RectangleShape thrusterLeft;
    sf::RectangleShape thrusterLeftBottom;
    sf::RectangleShape thrusterRight;
    sf::RectangleShape thrusterRightBottom;

    sf::Color cBody;
    sf::Color cCenter;
    sf::Color cThrusterOn;
    sf::Color cThrusterOff;

public:
    Renderer() {
        cBody = sf::Color(150,150,150);
        cCenter = sf::Color::Green;
        cThrusterOn = sf::Color::Yellow;
        cThrusterOff = sf::Color::White;

        base = sf::RectangleShape(sf::Vector2f{100, 20});
        base.setFillColor(cBody);
        base.setOrigin(sf::Vector2f{50.0,10.0});

        center = sf::CircleShape(20);
        center.setFillColor(cCenter);
        center.setOrigin(sf::Vector2f{20,20});

        thrusterLeft = sf::RectangleShape(sf::Vector2f{15, 40});
        thrusterLeft.setFillColor(sf::Color::Red);
        thrusterLeft.setOrigin(sf::Vector2f{7.5, 20});

        thrusterRight = sf::RectangleShape(sf::Vector2f{15,40});
        thrusterRight.setFillColor(sf::Color::Blue);
        thrusterRight.setOrigin(sf::Vector2f{7.5, 20});

        thrusterLeftBottom = sf::RectangleShape(sf::Vector2f{15,5});
        thrusterLeftBottom.setFillColor(cThrusterOff);
        thrusterLeftBottom.setOrigin(sf::Vector2f{7.5, -10});

        thrusterRightBottom = sf::RectangleShape(sf::Vector2f{15,5});
        thrusterRightBottom.setFillColor(cThrusterOff);
        thrusterRightBottom.setOrigin(sf::Vector2f{7.5, -10});
    }

	void draw(const Drone *drone, sf::RenderTarget& target, const sf::RenderStates& state) {
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

        target.draw(base, state);
        target.draw(center, state);
        target.draw(thrusterLeft, state);
        target.draw(thrusterRight, state);
        target.draw(thrusterLeftBottom, state);
        target.draw(thrusterRightBottom, state);
    }
};
