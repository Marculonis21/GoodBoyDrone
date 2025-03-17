#pragma once

#include <SFML/Graphics/CircleShape.hpp>
#include <SFML/Graphics/RenderStates.hpp>
#include <SFML/Graphics/RenderWindow.hpp>
#include <SFML/Window/ContextSettings.hpp>
#include <cstdio>
#include <memory>
#include <vector>

#include "drone.hpp"
#include "ea.hpp"
#include "renderer.hpp"
#include "utils.hpp"

struct AbstractRunner {
	int currentLevel = 0;
	std::vector<World> worldLevels;

	bool debugFlag = false;
	bool saveFlag = false;
	bool updateDoneFlag = false; 

	virtual void prepare(std::vector<World> levels) = 0;
	virtual void run(Drone &drone, Net &mother, EA &ea) = 0;

	void saveProcedure(EA &ea, Net &mother) {
		std::cout << "SAVING..." << std::endl;

		int64_t timestamp = std::chrono::system_clock::now().time_since_epoch().count();
		ea.saveEA("saves/ea_save_" + std::to_string(ea.generation) + "_" + std::to_string(timestamp) + ".json");   
		mother.saveConfig("saves/net_save_" + std::to_string(ea.generation) + "_" + std::to_string(timestamp) + ".json");   

		std::cout << "ALL SAVED" << std::endl;
	}

	void levelUpProcedure(EA &ea) {
		if (ea.lastMaxFitness > 90000 && currentLevel < worldLevels.size()) {
			currentLevel += 1;
		}
	}
};

struct ConsoleRunner : public AbstractRunner {
	void prepare(std::vector<World> levels) override {
		currentLevel = 0;
		worldLevels = levels;
	}

	void run(Drone &drone, Net &mother, EA &ea) override {

		while (true) 
		{
			// EA LOGIC
			updateDoneFlag = ea.update(dt, worldLevels[currentLevel], ea.generation % 500 == 0);

			// if at the end ea sim was finished, do the EA process, reset and the timing
			if (updateDoneFlag) {
				updateDoneFlag = false;

				ea.process_without_crossover();

				printf("Gen: %lu  Best Fitness: %.3f  Average Fitness: %.3f\n", ea.generation, ea.lastMaxFitness, ea.lastAverageFitness);
				// level up condition
			
				levelUpProcedure(ea);

				if (ea.generation % 1000 == 0) {
				    saveProcedure(ea, mother);
				}
			}
		}
	}
};

struct EAWindowRunner : public AbstractRunner {
	std::unique_ptr<sf::RenderWindow> window;
	std::unique_ptr<sf::RenderStates> state;

	std::unique_ptr<sf::CircleShape> wallPrefab;
	std::unique_ptr<sf::CircleShape> goalPrefab;

	std::unique_ptr<Renderer> renderer;

	void prepare(std::vector<World> levels) override {
		currentLevel = 0;
		levels = levels;

		// window prep
		sf::ContextSettings settings;
		settings.antialiasingLevel = 4;
		window = std::make_unique<sf::RenderWindow>(sf::VideoMode(winWidth, winHeight), "SFML - GoodBoyDrone", sf::Style::Default, settings);

		window->setMouseCursorVisible(true);
		window->setVerticalSyncEnabled(true);

		wallPrefab = std::make_unique<sf::CircleShape>();
		wallPrefab->setFillColor(sf::Color(55,55,55));

		goalPrefab = std::make_unique<sf::CircleShape>(10);
		goalPrefab->setOrigin(goalPrefab->getRadius(), goalPrefab->getRadius());
		goalPrefab->setFillColor(sf::Color(240,190,4));

		renderer = std::make_unique<Renderer>();
	}

	void run(Drone &drone, Net &mother, EA &ea) override {
		sf::Event event;
		while (window->isOpen()) 
		{
			// EVENTS
			while (window->pollEvent(event)) {
				if ((event.type == sf::Event::Closed) ||
					((event.type == sf::Event::KeyPressed) && (event.key.code == sf::Keyboard::Escape))) {
					window->close();
					break;
				}
				if ((event.type == sf::Event::KeyPressed) && (event.key.code == sf::Keyboard::F1)) {
					debugFlag = false;
				}
				if ((event.type == sf::Event::KeyPressed) && (event.key.code == sf::Keyboard::F2)) {
					debugFlag = true;
				}
				if ((event.type == sf::Event::KeyPressed) && (event.key.code == sf::Keyboard::Enter)) {
					saveFlag = true;
					std::cout << "SAVE STATE to TRUE" << std::endl;
				}
			}

			window->clear();

			for (auto && w : worldLevels[currentLevel].walls) {
				wallPrefab->setPosition(w.pos);
				wallPrefab->setRadius(w.radius);
				wallPrefab->setOrigin(w.radius,w.radius);

				window->draw(*wallPrefab);
			}

			/* sf::Vector2i mp = sf::Mouse::getPosition(window); */
			/* drone.pos = sf::Vector2f{mp}; */

			// EA LOGIC
			updateDoneFlag = ea.update(dt, worldLevels[currentLevel], ea.generation % 500 == 0);

			if (ea.generation % 500 == 0) {
				goalPrefab->setPosition(worldLevels[currentLevel].goals[ea.agents.at(0)->goalIndex % worldLevels[currentLevel].goals.size()]);
				renderer->draw_body(ea.agents.at(0).get(), *window, *state);
				if (debugFlag) {
					renderer->draw_debug(ea.agents.at(0).get(), worldLevels[currentLevel].walls, {}, *window, *state);
				}

				window->draw(*goalPrefab);
				window->display();
			}

			// SINGLE LOGIC
			/* drone.update(dt, boundary, walls); */
			/* renderer.draw_body(&drone, window, state); */
			/* if (drawDebug) { */
			/*     renderer.draw_debug(&drone, walls, {}, window, state); */
			/* } */

			// if at the end ea sim was finished, do the EA process, reset and the timing
			if (updateDoneFlag) {
				updateDoneFlag = false;

				ea.process_without_crossover();

				printf("Gen: %lu  Best Fitness: %.3f  Average Fitness: %.3f\n", ea.generation, ea.lastMaxFitness, ea.lastAverageFitness);
				levelUpProcedure(ea);

				if (saveFlag) {
					saveFlag = false;

					saveProcedure(ea, mother);
				}
			}
		}
	}
};
