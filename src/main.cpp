#include "drone.hpp"
#include "ea.hpp"
#include "net.hpp"
#include "renderer.hpp"
#include <SFML/Graphics/CircleShape.hpp>
#include <SFML/Graphics/Color.hpp>
#include <SFML/Graphics/RectangleShape.hpp>
#include <SFML/Graphics/RenderWindow.hpp>
#include <SFML/Graphics/View.hpp>
#include <SFML/System/Clock.hpp>
#include <SFML/System/Vector2.hpp>
#include <SFML/Window/Event.hpp>
#include <SFML/Window/Keyboard.hpp>
#include <SFML/Window/Mouse.hpp>
#include <SFML/Graphics.hpp>
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

int main(int argc, char* argv[]) {
    const uint32_t win_width = 800;
	const uint32_t win_height = 800;
	sf::ContextSettings settings;
	settings.antialiasingLevel = 4;
	sf::RenderWindow window(sf::VideoMode(win_width, win_height), "SFML - GoodBoyDrone", sf::Style::Default, settings);
    sf::RenderStates state;

    window.setMouseCursorVisible(false);

    sf::CircleShape cursor{10};
    cursor.setOrigin(10,10);
    cursor.setFillColor(sf::Color::White);

    sf::Vector2f goalPos{200,200};
    sf::CircleShape goal{10};
    goal.setOrigin(10,10);
    goal.setFillColor(sf::Color(240,190,4));
    goal.setPosition(goalPos);

    Drone drone{sf::Vector2f(400,400)};
    Renderer renderer;

    Net mother;
    mother.modules = {
        std::make_shared<Linear>(9, 16),
        std::make_shared<ReLU>(16),
        std::make_shared<Linear>(16, 4),
        std::make_shared<Tanh>(4),
    };

    mother.initialize();

    EA ea{5, mother, drone};

    sf::Clock clock;
    sf::Time elapsed = clock.restart();
    const sf::Time update_ms = sf::seconds(1.f / 120.f);
    while (window.isOpen()) 
    {
        // EVENTS
        sf::Event event;
        while (window.pollEvent(event)) {
            if ((event.type == sf::Event::Closed) ||
                    ((event.type == sf::Event::KeyPressed) && (event.key.code == sf::Keyboard::Escape))) {
                window.close();
                break;
            }

            // MOUSE CONTROLS - I SUCK! 
            /* if (event.type == sf::Event::MouseButtonPressed) { */
            /*     std::cout << "pressed" << std::endl; */
            /*     drone.thrusterLeft.powerController = 0.5; */
            /*     drone.thrusterRight.powerController = 0.5; */
            /* } */
            /* else if (event.type == sf::Event::MouseButtonReleased) { */
            /*     std::cout << "released" << std::endl; */
            /*     drone.thrusterLeft.powerController = 0; */
            /*     drone.thrusterRight.powerController = 0; */
            /* } */

            // TORQUE TESTS
            /* if ((event.type == sf::Event::KeyPressed) && (event.key.code == sf::Keyboard::Left)) { */
            /*     drone.thrusterLeft.powerController = 1; */
            /*     /1* drone.thrusterRight.powerController = 1; *1/ */
            /* } */
            /* else if ((event.type == sf::Event::KeyPressed) && (event.key.code == sf::Keyboard::Right)) { */
            /*     /1* drone.thrusterLeft.powerController = 1; *1/ */
            /*     drone.thrusterRight.powerController = 1; */
            /* } */
            /* else { */
            /*     drone.thrusterLeft.powerController = 0; */
            /*     drone.thrusterRight.powerController = 0; */
            /* } */
        }

        window.clear();

        /* auto mp = sf::Mouse::getPosition(window); */
        /* cursor.setPosition(mp.x, mp.y); */
        /* window.draw(cursor); */

        // TESTING THRUSTER ANGLECONTROLLER CONTROLS 
        /* const sf::Vector2f lheading = sf::Vector2f{mp} - (drone.pos - drone.thrusterOffset); */
        /* const sf::Vector2f rheading = sf::Vector2f{mp} - (drone.pos + drone.thrusterOffset); */
        /* const float lheadingAngle = atan2(lheading.y, lheading.x) + M_PI * 0.5f; */
        /* const float rheadingAngle = atan2(rheading.y, rheading.x) + M_PI * 0.5f; */
        /* drone.thrusterLeft.angleController = lheadingAngle - drone.angle; */
        /* drone.thrusterRight.angleController = rheadingAngle - drone.angle; */

        // LOGIC
        elapsed += clock.restart();
        while (elapsed >= update_ms) 
        {
            ea.update(elapsed.asSeconds(), sf::Vector2f{win_width, win_height});
            elapsed -= update_ms;
        }

        goal.setPosition(ea.goals[ea.agents[0].get()->goalIndex]);
        renderer.draw(ea.agents[0].get(), window, state);
        window.draw(goal);
        window.display();

        // if at the end ea sim was finished, do the EA process, reset and the timing
        if (ea.simFinished) {
            ea.process();

            elapsed = sf::Time::Zero;
            clock.restart();
        }
    }
    
    return 0;
}
