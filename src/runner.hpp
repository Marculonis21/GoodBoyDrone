#pragma once

#include <SFML/Graphics/CircleShape.hpp>
#include <SFML/Graphics/RenderStates.hpp>
#include <SFML/Graphics/RenderWindow.hpp>
#include <SFML/System/Vector2.hpp>
#include <SFML/Window/ContextSettings.hpp>
#include <SFML/Window/Mouse.hpp>
#include <cstdio>
#include <exception>
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

	virtual void prepare(const std::vector<World> &levels) = 0;
	virtual void run(Drone &drone, Net &mother, EA &ea) = 0;

	void saveProcedure(const EA &ea, const Net &mother) {
		std::cout << "SAVING..." << std::endl;

		int64_t timestamp = std::chrono::system_clock::now().time_since_epoch().count();
		ea.saveEA("saves/ea_save_" + std::to_string(ea.generation) + "_" + std::to_string(timestamp) + ".json");   
		mother.saveConfig("saves/net_save_" + std::to_string(ea.generation) + "_" + std::to_string(timestamp) + ".json");   

		std::cout << "ALL SAVED" << std::endl;
	}

	void levelUpProcedure(const EA &ea) {
		/* if (ea.lastMaxFitness > 50000 && currentLevel < worldLevels.size()) { */
		/* 	currentLevel += 1; */
		/* } */
	}

	void debugPrintProcedure(const EA &ea) { 
		printf("Gen: %lu Lvl: %d --- BF: %.3f AVGF: %.3f\n", ea.generation, currentLevel, ea.lastMaxFitness, ea.lastAverageFitness);
	}
};

struct ConsoleRunner : public AbstractRunner {
	void prepare(const std::vector<World> &levels) override {
		currentLevel = 0;
		worldLevels = levels;
	}

	void run(Drone &drone, Net &mother, EA &ea) override {

		while (true) 
		{
			// EA LOGIC
			updateDoneFlag = ea.update(dt, worldLevels[currentLevel], false);

			// if at the end ea sim was finished, do the EA process, reset and the timing
			if (updateDoneFlag) {
				updateDoneFlag = false;

				ea.process_without_crossover();

				debugPrintProcedure(ea);

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

	std::unique_ptr<sf::CircleShape> wallPrefab;
	std::unique_ptr<sf::CircleShape> goalPrefab;

	std::unique_ptr<Renderer> renderer;

	virtual void prepare(const std::vector<World> &levels) override {
		currentLevel = 0;
		worldLevels = levels;

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

			/* sf::Vector2i mp = sf::Mouse::getPosition(*window); */
			/* drone.pos = sf::Vector2f{mp}; */

			// EA LOGIC
			
			// debug run
			updateDoneFlag = ea.update(dt, worldLevels[currentLevel], ea.generation % 500 == 0);

			if (ea.generation % 500 == 0) {
				goalPrefab->setPosition(worldLevels[currentLevel].goals[ea.agents.at(0)->goalIndex % worldLevels[currentLevel].goals.size()]);
				renderer->draw_body(ea.agents.at(0).get(), window.get());
				if (debugFlag) {
					renderer->draw_debug(ea.agents.at(0).get(), worldLevels[currentLevel], {}, window.get());
				}

				window->draw(*goalPrefab);
				window->display();
			}

			// SINGLE LOGIC
			/* drone.update(dt, worldLevels[currentLevel]); */
			/* renderer->draw_body(&drone, window.get()); */
			/* if (debugFlag) { */
			/*     renderer->draw_debug(&drone,worldLevels[currentLevel], {}, window.get()); */
			/* } */
			/* std::vector<float> obs(17); */
			/* drone.genObservation_with_sensors(obs, worldLevels[currentLevel]); */
			/* window->draw(*goalPrefab); */
			/* window->display(); */

			// if at the end ea sim was finished, do the EA process, reset and the timing
			if (updateDoneFlag) {
				updateDoneFlag = false;

				ea.process_without_crossover();

				debugPrintProcedure(ea);
				levelUpProcedure(ea);

				if (saveFlag) {
					saveFlag = false;

					saveProcedure(ea, mother);
				}
			}
		}
	}
};

struct HumanRunner : public EAWindowRunner {

	sf::Vector2f mousePos;

	void prepare(const std::vector<World> &levels) override {
		EAWindowRunner::prepare(levels);
	}

	void run(Drone &drone, Net &mother, EA &ea) override {
		sf::Event event;

		Drone *runnerDrone = ea.agents[0].get();
		Net *runnerNet = ea.population[0].get();
		World runnerWorld = worldLevels[currentLevel];

		std::vector<float> observation;
		observation.resize(13);

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
				if ((event.type == sf::Event::KeyPressed) && (event.key.code == sf::Keyboard::Space)) {
					runnerDrone->reset();
				}
			}

			window->clear();

			for (auto && w : worldLevels[currentLevel].walls) {
				wallPrefab->setPosition(w.pos);
				wallPrefab->setRadius(w.radius);
				wallPrefab->setOrigin(w.radius,w.radius);
				window->draw(*wallPrefab);
			}

			// set the goal under cursosr
			mousePos = sf::Vector2f(sf::Mouse::getPosition(*window));
			runnerWorld.goals[0] = mousePos;

			// don't let the drone die
			runnerDrone->aliveTimer = 0;
			runnerDrone->update(dt, runnerWorld);

			runnerDrone->genObservation_with_sensors(observation, runnerWorld);
			Output controls = runnerNet->predict(observation);
			runnerDrone->control(controls[0], controls[1], controls[2], controls[3]);

			renderer->draw_body(runnerDrone, window.get());
			if (debugFlag) {
				renderer->draw_debug(runnerDrone, runnerWorld, {}, window.get());
			}

			goalPrefab->setPosition(mousePos);
			window->draw(*goalPrefab);
			window->display();
		}
	}
};

