#pragma once

#include "ea.hpp"
#include "easyea.hpp"
#include "cosyne.hpp"
#include <memory>
#include <string>

struct Loader{
	static std::unique_ptr<AbstractEA> loadEA(const std::string &path, const Drone &father) {
		assert(std::filesystem::exists(path) && "Path selected for loading EA does not exist!");

		std::cout << "LOADING FROM "<< path << std::endl;

        std::ifstream input(path);
        json config;
        input >> config;
        input.close();

		std::string type = config["type"];

        size_t popSize = config["popSize"];
		Net mother = Net::loadConfig(config["motherNet"]);
        mother.initialize();

		std::unique_ptr<AbstractEA> loaded;
		if (type == "EasyEA") {
			loaded = std::make_unique<EasyEA>(popSize, mother, father);
		}
		else if (type == "CoSyNE") {
			loaded = std::make_unique<CoSyNE>(popSize, mother, father);
		}
		else {
			throw std::invalid_argument("Unknown EA load type encountered");
		}

		loaded->loadPopW(config);

        return loaded;
	}
};
