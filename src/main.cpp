#include "drone.hpp"
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
#include <iostream>

int main(int argc, char* argv[]) {
    const uint32_t win_width = 800;
	const uint32_t win_height = 800;
	sf::ContextSettings settings;
	settings.antialiasingLevel = 4;
	sf::RenderWindow window(sf::VideoMode(win_width, win_height), "SFML - GoodBoyDrone", sf::Style::Default, settings);

    window.setMouseCursorVisible(false);

    sf::CircleShape circle{10};
    circle.setOrigin(10,10);
    circle.setFillColor(sf::Color::White);

    Drone drone{sf::Vector2f(400,400)};
    Renderer renderer;

    sf::RenderStates state;

    sf::Clock clock;
    sf::Time elapsed = clock.restart();
    const sf::Time update_ms = sf::seconds(1.f / 60.f);
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

            if ((event.type == sf::Event::KeyPressed) && (event.key.code == sf::Keyboard::Left)) {
                drone.thrusterLeft.powerController = 1;
                /* drone.thrusterRight.powerController = 1; */
            }
            else if ((event.type == sf::Event::KeyPressed) && (event.key.code == sf::Keyboard::Right)) {
                /* drone.thrusterLeft.powerController = 1; */
                drone.thrusterRight.powerController = 1;
            }
            else {
                drone.thrusterLeft.powerController = 0;
                drone.thrusterRight.powerController = 0;
            }
        }

        auto mp = sf::Mouse::getPosition(window);
        circle.setPosition(mp.x, mp.y);

        // TESTING THRUSTER ANGLECONTROLLER CONTROLS 
        /* const sf::Vector2f lheading = sf::Vector2f{mp} - (drone.pos - drone.thrusterOffset); */
        /* const sf::Vector2f rheading = sf::Vector2f{mp} - (drone.pos + drone.thrusterOffset); */
        /* const float lheadingAngle = atan2(lheading.y, lheading.x) + M_PI * 0.5f; */
        /* const float rheadingAngle = atan2(rheading.y, rheading.x) + M_PI * 0.5f; */
        /* drone.thrusterLeft.angleController = lheadingAngle - drone.angle; */
        /* drone.thrusterRight.angleController = rheadingAngle - drone.angle; */

        /* drone.thrusterLeft.angleController = 0.9; */
        /* drone.thrusterRight.angleController = 0; */

        // LOGIC
        elapsed += clock.restart();
        while (elapsed >= update_ms) 
        {
            /* drone.thrusterRight.angleController */
            drone.update(elapsed.asSeconds());
            elapsed -= update_ms;
        }

        window.clear();

        window.draw(circle);

        renderer.draw(drone, window, state);

        window.display();
    }
    
    return 0;
}
