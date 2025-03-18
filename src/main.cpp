#include "drone.hpp"
#include "ea.hpp"
#include "net.hpp"
#include <cassert>
#include <cmath>
#include <memory>
#include <string>

#include "runner.hpp"
#include "utils.hpp"

int main(int argc, char* argv[]) {


    assert(argc > 1 && "We expect more than one argument for Runner");

    World world {
        .boundary = sf::Vector2f{winWidth,winHeight},
        .walls = {},
        .goals = {
			sf::Vector2f{200, 200}, 
			sf::Vector2f{600, 600}, 
			sf::Vector2f{200, 600},
			sf::Vector2f{600, 200},
			sf::Vector2f{400, 650}
		}
    };

    const World world_lvl2 {
        world.boundary,
        std::vector<Wall>{ 
            Wall{sf::Vector2f{400, 400}, 100},
            Wall{sf::Vector2f{300, 400}, 50},
        },
        world.goals
    };

    Drone drone{sf::Vector2f(400,650)};

    Net mother;
    mother.modules.push_back(std::make_unique<Linear>(13, 32));
    mother.modules.push_back(std::make_unique<Tanh>(32));
    mother.modules.push_back(std::make_unique<Linear>(32, 8));
    mother.modules.push_back(std::make_unique<Tanh>(8));
    mother.modules.push_back(std::make_unique<Linear>(8, 4));
    mother.modules.push_back(std::make_unique<Tanh>(4));
    mother.initialize();

    EA ea{250, mother, drone};

    std::unique_ptr<AbstractRunner> runner;
    if (std::string(argv[1]) == "window") {
        runner = std::make_unique<EAWindowRunner>();
    }
    else if (std::string(argv[1]) == "console") {
        runner = std::make_unique<ConsoleRunner>();
    }
    else if (std::string(argv[1]) == "human") {
        assert(argc == 4 && "For Human run please include ea and net config file");
        runner = std::make_unique<HumanRunner>();
        std::string eaFile = argv[2];
        std::string netFile = argv[3];

        Net humanMOTHER = Net::loadConfig(netFile);

        EA humanEA{250, humanMOTHER, drone};
        humanEA.loadPopEA(eaFile);

        runner->prepare(std::vector<World>{world, world_lvl2});
        runner->run(drone, humanMOTHER, humanEA);

        return 0;
    }
    else {
        return 1;
    }
    
    runner->prepare(std::vector<World>{world, world_lvl2});
    runner->run(drone, mother, ea);

    return 0;
}
