#include "cosyne.hpp"
#include "drone.hpp"
#include "ea.hpp"
#include "easyea.hpp"
#include "loader.hpp"
#include "net.hpp"
#include <cassert>
#include <cmath>
#include <memory>
#include <string>

#include "runner.hpp"
#include "utils.hpp"

int main(int argc, char *argv[]) {

	assert(argc > 2 && "We expect at least 1 args - <runner type>");

	Drone drone{droneStart};

	std::unique_ptr<AbstractRunner> runner;
	std::unique_ptr<AbstractEA> ea;

	if (std::string(argv[1]) == "human") {
		runner = std::make_unique<HumanRunner>();

		assert(argc == 3 && "For Human run please include ea save config file");
		std::string eaConfig = argv[2];

		ea = Loader::loadEA(eaConfig, drone);
	} else if (std::string(argv[1]) == "window") {
		runner = std::make_unique<EAWindowRunner>();
	} else if (std::string(argv[1]) == "console") {
		runner = std::make_unique<ConsoleRunner>();
	} else {
		std::cout << "Incorrect runner selected - possible: 'window', 'console', 'human'" << std::endl;
		return 1;
	}

	Net mother;
	mother.modules.push_back(std::make_unique<Linear>(8, 16));
	mother.modules.push_back(std::make_unique<Tanh>(16));
	mother.modules.push_back(std::make_unique<Linear>(16, 4));
	mother.modules.push_back(std::make_unique<Tanh>(4));
	mother.initialize();

	if (std::string(argv[1]) != "human") {
		assert(argc == 3 && "For window/console run please include ea type - 'easyea', 'cosyne'");

		if (std::string(argv[2]) == "easyea") {
			ea = std::make_unique<EasyEA>(128, mother, drone);
		} else if (std::string(argv[2]) == "cosyne") {
			ea = std::make_unique<CoSyNE>(256, mother, drone);
		} else {
			std::cout << "Incorrect ea selected - possible: 'easyea', 'cosyne'"
					  << std::endl;
			return 1;
		}
	}

	const World world{
		.boundary = sf::Vector2f{winWidth, winHeight},
	    .walls = {},
	    .goals = {sf::Vector2f{200, 200}, sf::Vector2f{600, 600},
	      		  sf::Vector2f{200, 600}, sf::Vector2f{600, 200},
	      		  sf::Vector2f{400, 650}},
	    .isStatic = true,
	};

	const World world_randomized{
		.boundary = sf::Vector2f{winWidth, winHeight},
	    .walls = {},
	    .goals = {sf::Vector2f{200, 200}, sf::Vector2f{600, 600},
	      		  sf::Vector2f{200, 600}, sf::Vector2f{600, 200},
	      		  sf::Vector2f{400, 650}},
	    .isStatic = false,
	};

	const World world_lvl2{
		world.boundary,
		std::vector<Wall>{
			Wall{sf::Vector2f{400, 400}, 100},
			Wall{sf::Vector2f{300, 400}, 50},
		},
		world.goals,
		true,
	};

	const World world_lvl2_randomized{
		world.boundary,
		std::vector<Wall>{
			Wall{sf::Vector2f{400, 400}, 100},
			Wall{sf::Vector2f{300, 400}, 50},
		},
		world.goals,
		false,
	};

	runner->prepare(std::vector<World>{world, world_randomized, world_lvl2, world_lvl2_randomized});
	/* runner->prepare(std::vector<World>{world, world_lvl2}); */
	runner->run(drone, std::move(ea), -1);

	return 0;
}
