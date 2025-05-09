#include "cosyne.hpp"
#include "drone.hpp"
#include "ea.hpp"
#include "easyea.hpp"
#include "net.hpp"
#include <cassert>
#include <cmath>
#include <memory>
#include <string>

#include "runner.hpp"
#include "utils.hpp"

int main(int argc, char *argv[]) {

	assert(argc == 3 && "We expect 2 args - <runner type> <ea type>");

	Drone drone{droneStart};

	Net mother;
	mother.modules.push_back(std::make_unique<Linear>(10, 16));
	mother.modules.push_back(std::make_unique<Tanh>(16));
	mother.modules.push_back(std::make_unique<Linear>(16, 4));
	mother.modules.push_back(std::make_unique<Tanh>(4));
	mother.initialize();

	std::unique_ptr<AbstractEA> ea;
	if (std::string(argv[2]) == "easyea") {
		ea = std::make_unique<EasyEA>(128, mother, drone);
	} else if (std::string(argv[2]) == "cosyne") {
		ea = std::make_unique<CoSyNE>(128, mother, drone);
	} else {
		std::cout << "Incorrect ea selected - possible: 'easyea', 'cosyne'"
				  << std::endl;
		return 1;
	}

	std::unique_ptr<AbstractRunner> runner;
	if (std::string(argv[1]) == "window") {
		runner = std::make_unique<EAWindowRunner>();
	} else if (std::string(argv[1]) == "console") {
		runner = std::make_unique<ConsoleRunner>();
	} else if (std::string(argv[1]) == "human") {
		return 404;

		/* assert(argc == 4 && "For Human run please include ea and net config
		 * file"); */
		/* runner = std::make_unique<HumanRunner>(); */
		/* std::string eaFile = argv[2]; */
		/* std::string netFile = argv[3]; */

		/* mother = Net::loadConfig(netFile); */

		/* /1* std::unique_ptr<AbstractEA> humanEA humanEA{1, mother, drone};
		 * *1/ */
		/* humanEA.loadPopEA(eaFile); */

		/* runner->prepare(std::vector<World>{world, world_lvl2}); */
		/* runner->run(drone, mother, humanEA); */

		/* return 0; */
	} else {
		std::cout << "Incorrect runner selected - possible: 'window', "
					 "'console', 'human'"
				  << std::endl;
		return 1;
	}

	const World world{.boundary = sf::Vector2f{winWidth, winHeight},
					  .walls = {},
					  .goals = {sf::Vector2f{200, 200}, sf::Vector2f{600, 600},
								sf::Vector2f{200, 600}, sf::Vector2f{600, 200},
								sf::Vector2f{400, 650}},
					  .isStatic = true};

	const World world_lvl2{
		world.boundary,
		std::vector<Wall>{
			Wall{sf::Vector2f{400, 400}, 100},
			Wall{sf::Vector2f{300, 400}, 50},
		},
		world.goals,
		true,
	};

	const World world_lvl3{
		world.boundary,
		std::vector<Wall>{
			Wall{sf::Vector2f{400, 400}, 100},
			Wall{sf::Vector2f{400, 400}, 100},
			Wall{sf::Vector2f{400, 400}, 100},
		},
		world.goals,
		false,
	};

	runner->prepare(std::vector<World>{world, world_lvl2, world_lvl3});
	runner->run(drone, mother, std::move(ea), -1);

	return 0;
}
