#include "drone.hpp"
#include "ea.hpp"
#include "net.hpp"
#include "renderer.hpp"
#include <SFML/Graphics/CircleShape.hpp>
#include <SFML/Graphics/Color.hpp>
#include <SFML/Graphics/RectangleShape.hpp>
#include <SFML/Graphics/RenderWindow.hpp>
#include <SFML/Graphics/View.hpp>
#include <SFML/System/Vector2.hpp>
#include <SFML/Window/Event.hpp>
#include <SFML/Window/Keyboard.hpp>
#include <SFML/Window/Mouse.hpp>
#include <SFML/Graphics.hpp>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <functional>
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
    window.setVerticalSyncEnabled(true);
    /* window.setFramerateLimit(6); */

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
    mother.modules.push_back(std::make_unique<Linear>(9, 10));
    mother.modules.push_back(std::make_unique<ReLU  >(10));
    mother.modules.push_back(std::make_unique<Linear>(10, 4));
    mother.modules.push_back(std::make_unique<Tanh  >(4));

    mother.initialize();

    EA ea{6, mother, drone};
    std::cout << "EA DONE" << std::endl;

    constexpr float dt = 1.f / 60.f;
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
        }

        window.clear();

        // LOGIC
        ea.update(dt, sf::Vector2f{win_width, win_height});

        goal.setPosition(ea.goals[ea.agents[0].get()->goalIndex]);
        renderer.draw(ea.agents[0].get(), window, state);
        /* renderer.draw(&drone, window, state); */
        window.draw(goal);
        window.display();

        // if at the end ea sim was finished, do the EA process, reset and the timing
        if (ea.simFinished) {
            ea.process();
            std::cout << "EA DONE" << std::endl;
        }
    }
    
    return 0;
}
