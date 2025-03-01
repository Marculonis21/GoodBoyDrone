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

    const sf::Vector2f boundary{win_width, win_height};

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
    mother.modules.push_back(std::make_unique<Linear>(9, 16));
    mother.modules.push_back(std::make_unique<Tanh>(16));
    /* mother.modules.push_back(std::make_unique<Linear>(16, 16)); */
    /* mother.modules.push_back(std::make_unique<Tanh>(8)); */
    mother.modules.push_back(std::make_unique<Linear>(16, 4));
    mother.modules.push_back(std::make_unique<Tanh  >(4));

    mother.initialize();

    EA ea{100, mother, drone};
    std::cout << "EA DONE" << std::endl;
    size_t generation = 0;

    constexpr float dt = 1.f / 60.f;
    sf::Event event;
    while (window.isOpen()) 
    {
        // EVENTS
        while (window.pollEvent(event)) {
            if ((event.type == sf::Event::Closed) ||
                    ((event.type == sf::Event::KeyPressed) && (event.key.code == sf::Keyboard::Escape))) {
                window.close();
                break;
            }
        }

        window.clear();

        // EA LOGIC
        ea.update(dt, boundary, generation % 500 == 0);

        goal.setPosition(ea.goals[ea.agents.at(0)->goalIndex]);
        renderer.draw(ea.agents.at(0).get(), window, state);
        window.draw(goal);

        // SINGLE LOGIC
        /* drone.update(dt, boundary); */
        /* renderer.draw(&drone, window, state); */

        if (generation % 500 == 0) {
            window.display();
        }

        // if at the end ea sim was finished, do the EA process, reset and the timing
        if (ea.simFinished) {
            std::cout << "EA PROCESS START" << std::endl;
            ea.process();
            std::cout << "EA DONE" << std::endl;
            generation += 1;
            std::cout << "Gen: " << generation << std::endl;
        }
    }
    
    return 0;
}
