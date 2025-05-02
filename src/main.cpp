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

int main(int argc, char* argv[]) {

    assert(argc > 1 && "We expect more than one argument for Runner");

    Drone drone{sf::Vector2f(400,650)};

    Net mother;
    mother.modules.push_back(std::make_unique<Linear>(7, 16));
    mother.modules.push_back(std::make_unique<Tanh>(16));
    /* mother.modules.push_back(std::make_unique<Linear>(16, 8)); */
    /* mother.modules.push_back(std::make_unique<Tanh>(8)); */
    mother.modules.push_back(std::make_unique<Linear>(16, 4));
    mother.modules.push_back(std::make_unique<Tanh>(4));
    mother.initialize();


    /* std::unique_ptr<AbstractEA> easyea = std::make_unique<EasyEA>(250, mother, drone); */
    /* std::unique_ptr<AbstractEA> cosyne = std::make_unique<CoSyNE>(200, mother, drone); */
    /* std::unique_ptr<AbstractEA> ea = std::make_unique<CoSyNE>(256, mother, drone); */


    /* std::unique_ptr<AbstractRunner> runner; */
    /* if (std::string(argv[1]) == "window") { */
    /*     runner = std::make_unique<EAWindowRunner>(); */
    /* } */
    /* else if (std::string(argv[1]) == "console") { */
    /*     runner = std::make_unique<ConsoleRunner>(); */
    /* } */
    /* else if (std::string(argv[1]) == "human") { */
    /*     return 404; */

    /*     /1* assert(argc == 4 && "For Human run please include ea and net config file"); *1/ */
    /*     /1* runner = std::make_unique<HumanRunner>(); *1/ */
    /*     /1* std::string eaFile = argv[2]; *1/ */
    /*     /1* std::string netFile = argv[3]; *1/ */

    /*     /1* mother = Net::loadConfig(netFile); *1/ */

    /*     /1* /2* std::unique_ptr<AbstractEA> humanEA humanEA{1, mother, drone}; *2/ *1/ */
    /*     /1* humanEA.loadPopEA(eaFile); *1/ */

    /*     /1* runner->prepare(std::vector<World>{world, world_lvl2}); *1/ */
    /*     /1* runner->run(drone, mother, humanEA); *1/ */

    /*     /1* return 0; *1/ */
    /* } */
    /* else { */
    /*     return 1; */
    /* } */

    const World world {
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

    std::vector<float> mutprobs{0.025, 0.05, 0.1, 0.2};
    std::vector<float> mutcauchys{0.3, 0.5};

    std::unique_ptr<AbstractRunner> runner;
    for (int mp = 0; mp < mutprobs.size(); ++mp) {
        for (int mc = 0; mc < mutcauchys.size(); ++mc) {
            for (int i = 0; i < 5; ++i) {
                runner = std::make_unique<ConsoleRunner>();
                runner->prepare(std::vector<World>{world, world_lvl2});

                std::unique_ptr<AbstractEA> ea = std::make_unique<CoSyNE>(64, mother, drone, mutprobs[mp], mutcauchys[mc]);
                std::string note = "gsCoSyNE128_"+std::to_string(mutprobs[mp])+"_"
                                                 +std::to_string(mutcauchys[mc])+"_"
                                                 +"run"+std::to_string(i)+".csv";
                runner->run(drone, mother, std::move(ea), 1000, note);
            }
        }
    }


    return 0;
}
