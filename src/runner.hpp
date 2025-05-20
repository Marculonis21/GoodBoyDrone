#pragma once

#include <SFML/Graphics/CircleShape.hpp>
#include <SFML/Graphics/RenderStates.hpp>
#include <SFML/Graphics/RenderWindow.hpp>
#include <SFML/System/Vector2.hpp>
#include <SFML/Window/ContextSettings.hpp>
#include <SFML/Window/Mouse.hpp>
#include <cstdio>
#include <fstream>
#include <memory>
#include <string>
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
	virtual void run(Drone &drone, std::unique_ptr<AbstractEA> ea, const int maxGen=-1, const std::string &note="") = 0;

	void saveProcedure(const AbstractEA &ea, const std::string &note) const {
		std::cout << "SAVING..." << std::endl;

		int64_t timestamp = std::chrono::system_clock::now().time_since_epoch().count();
		ea.saveEA("ea_save_" + note + "_" + std::to_string(ea.generation) + "_" + std::to_string(ea.lastFitnessStats.max) + "_" + std::to_string(timestamp));   

		std::cout << "ALL SAVED" << std::endl;
	}

	void levelUpProcedure(const AbstractEA &ea) {
		if (ea.lastFitnessStats.max > 45000 && currentLevel+1 < worldLevels.size()) {
			/* std::cout << "[WARN] LEVEL UP DISSABLED!" << std::endl; */
			currentLevel += 1;
		}
	}

	void debugPrintProcedure(const AbstractEA &ea) const { 
		printf("Gen: %lu Lvl: %d --- BF: %.3f AVGF: %.3f\n", ea.generation, currentLevel, ea.lastFitnessStats.max, ea.lastFitnessStats.avg);
	}

	void fitStatsSave(const AbstractEA &ea, std::ofstream &os) const {
        os << std::to_string(ea.generation) + "," + std::to_string(ea.lastFitnessStats.max) + ","
		  										  + std::to_string(ea.lastFitnessStats.min) + ","
		  										  + std::to_string(ea.lastFitnessStats.avg) + ","
		  										  + std::to_string(ea.lastFitnessStats.med) + "," << std::endl;
	}
};

struct ConsoleRunner : public AbstractRunner {
	void prepare(const std::vector<World> &levels) override {
		currentLevel = 0;
		worldLevels = levels;
	}

	void run(Drone &drone, std::unique_ptr<AbstractEA> ea, const int maxGen, const std::string &note) override {

		/* std::string outpath = "fits/" + note + ".csv"; */
		/* std::ofstream os(outpath); */
		/* if (note == "") { */
		/* 	os.close(); */
		/* } */

		while ((maxGen > 0) ? (ea->generation < maxGen) : true) 
		{
			// EA LOGIC
			updateDoneFlag = ea->update(dt, worldLevels[currentLevel], false);

			// if at the end ea sim was finished, do the EA process, reset and the timing
			if (updateDoneFlag) {
				updateDoneFlag = false;

				ea->process();

				debugPrintProcedure(*ea);

				levelUpProcedure(*ea);

				if (ea->generation % 1000 == 0) {
				    saveProcedure(*ea, note);
				}

				/* if (os.is_open()) { */
				/* 	fitStatsSave(*ea, os); */
				/* } */
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
		window->setFramerateLimit(60);
		window->setVerticalSyncEnabled(true);

		wallPrefab = std::make_unique<sf::CircleShape>();
		wallPrefab->setFillColor(sf::Color(55,55,55));

		goalPrefab = std::make_unique<sf::CircleShape>(10);
		goalPrefab->setOrigin(goalPrefab->getRadius(), goalPrefab->getRadius());
		goalPrefab->setFillColor(sf::Color(240,190,4));

		renderer = std::make_unique<Renderer>();
	}

	void run(Drone &drone, std::unique_ptr<AbstractEA> ea, const int maxGen, const std::string &note) override {
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
			updateDoneFlag = ea->update(dt, worldLevels[currentLevel], ea->generation % 500 == 0);

			if (ea->generation % 500 == 0) {
				EAItem best = (*ea)[0];

				goalPrefab->setPosition(worldLevels[currentLevel].goals[best.drone->goalIndex % worldLevels[currentLevel].goals.size()]);
				renderer->draw_body(best.drone, window.get());
				if (debugFlag) {
					renderer->draw_debug(best.drone, worldLevels[currentLevel], {}, window.get());
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

				ea->process();
				worldLevels[currentLevel].randomize();

				debugPrintProcedure(*ea);

				levelUpProcedure(*ea);

				if (saveFlag) {
					saveFlag = false;

					saveProcedure(*ea, note);
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

	void run(Drone &drone, std::unique_ptr<AbstractEA> ea, const int maxGen, const std::string &note) override {
		sf::Event event;

		EAItem best = (*ea)[0];
		Drone *runnerDrone = best.drone;
		Net *runnerNet = best.net;

		World runnerWorld = worldLevels[currentLevel];

		std::vector<float> observation;
		observation.resize(ea->input_size);

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

			// WARN: THIS IS PROBLEMATIC
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

