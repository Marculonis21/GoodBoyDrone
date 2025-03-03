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
#include <cassert>
#include <cmath>

#include "utils.hpp"

int main(int argc, char* argv[]) {
	sf::ContextSettings settings;
	settings.antialiasingLevel = 4;
	sf::RenderWindow window(sf::VideoMode(winWidth, winHeight), "SFML - GoodBoyDrone", sf::Style::Default, settings);
    sf::RenderStates state;

    window.setMouseCursorVisible(true);
    window.setVerticalSyncEnabled(true);

    sf::CircleShape wallPrefab;
    wallPrefab.setFillColor(sf::Color(55,55,55));

    sf::CircleShape goal{10};
    goal.setOrigin(goal.getRadius(), goal.getRadius());
    goal.setFillColor(sf::Color(240,190,4));

    constexpr float dt = 1.f / 60.f;
    Drone drone{sf::Vector2f(400,650)};
    Renderer renderer;

    Net mother;
    /* mother.modules.push_back(std::make_unique<Linear>(13, 16)); */
    mother.modules.push_back(std::make_unique<Linear>(13, 16));
    mother.modules.push_back(std::make_unique<Tanh>(16));
    /* mother.modules.push_back(std::make_unique<Linear>(16, 16)); */
    /* mother.modules.push_back(std::make_unique<Tanh>(16)); */
    mother.modules.push_back(std::make_unique<Linear>(16, 4));
    mother.modules.push_back(std::make_unique<Tanh>(4));
    mother.initialize();

    EA ea{250, mother, drone};

    /* std::vector<Wall> walls { */
    /*     Wall{sf::Vector2f{400, 400}, 100}, */
    /*     Wall{sf::Vector2f{300, 400}, 50}, */
    /* }; */
    std::vector<Wall> walls {
    };

    size_t generation = 0;

    bool drawDebug = false;

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

            if ((event.type == sf::Event::KeyPressed) && (event.key.code == sf::Keyboard::F1)) {
                drawDebug = false;
            }
            if ((event.type == sf::Event::KeyPressed) && (event.key.code == sf::Keyboard::F2)) {
                drawDebug = true;
            }
        }

        window.clear();

        for (auto && w : walls) {
            wallPrefab.setPosition(w.pos);
            wallPrefab.setRadius(w.radius);
            wallPrefab.setOrigin(w.radius,w.radius);

            window.draw(wallPrefab);
        }

        /* sf::Vector2i mp = sf::Mouse::getPosition(window); */
        /* drone.pos = sf::Vector2f{mp}; */

        // EA LOGIC
        ea.update(dt, boundary, walls, generation % 500 == 0);

        if (generation % 500 == 0) {
            goal.setPosition(ea.goals[ea.agents.at(0)->goalIndex % ea.goals.size()]);
            renderer.draw_body(ea.agents.at(0).get(), window, state);
            if (drawDebug) {
                renderer.draw_debug(ea.agents.at(0).get(), walls, {}, window, state);
            }

            window.draw(goal);
            window.display();
        }

        // SINGLE LOGIC
        /* drone.update(dt, boundary, walls); */
        /* renderer.draw_body(&drone, window, state); */
        /* if (drawDebug) { */
        /*     renderer.draw_debug(&drone, walls, {}, window, state); */
        /* } */

        // if at the end ea sim was finished, do the EA process, reset and the timing
        if (ea.simFinished) {
            /* ea.process(); */
            ea.process_without_crossover();
            generation += 1;
            std::cout << "Gen: " << generation << " Best Fitness: " << ea.lastMaxFitness << std::endl;
            /* if (walls.size() == 0) { */
            /*     if (ea.lastMaxFitness > 30000) { */
            /*         walls = { */
            /*             Wall{sf::Vector2f{400, 400}, 100}, */
            /*             Wall{sf::Vector2f{300, 400}, 50}, */
            /*         }; */
            /*     } */
            /* } */
        }
    }
    
    return 0;
}
