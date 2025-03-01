#pragma once

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <memory>
#include <random>
#include <vector>
#include <iostream>

using Input = std::vector<float>;
using Output = std::vector<float>;
using Weights = std::vector<float>;

inline std::random_device rd;
inline std::mt19937 gen(time(nullptr));

struct Module {

    const std::size_t in;
    const std::size_t out;

    Weights weights = {};
    Output mockOutput = {};

    Module(std::size_t in, std::size_t out) : in(in), out(out) {
        mockOutput.resize(out);
    }

    virtual ~Module() {};

    virtual void initialize() = 0;
    virtual Output forward(const Input &input) = 0;
    virtual std::unique_ptr<Module> clone() const = 0;
};


struct Linear : public Module {
    Linear(std::size_t in, std::size_t out) : Module(in,out) {
        weights.resize(in*out + out);
    }

    void initialize() override {
        std::uniform_real_distribution<float> distr(-1.0f, 1.0f);
        for (int i = 0; i < out; ++i) {
            for (int j = 0; j < in; ++j) {
                weights[i*in + j] = distr(gen);
            }

            weights[in*out + i] = distr(gen);
        }
    }

    Output forward(const Input &input) override {
        for (int i = 0; i < out; ++i) {
            mockOutput[i] = 0;
            for (int j = 0; j < in; ++j) {
                mockOutput[i] += input[j]*weights[i*in + j];
            }

            mockOutput[i] += weights[in*out + i];
        }
        return mockOutput;
    }

    std::unique_ptr<Module> clone() const override {
        return std::make_unique<Linear>(*this);
    }
};

struct ReLU : public Module {
    ReLU(std::size_t out) : Module(0, out) {}

    void initialize() override {}

    Output forward(const Input &input) override {
        for (int i = 0; i < input.size(); ++i) {
            mockOutput[i] = std::max(0.0f, input[i]);
        }
        return mockOutput;
    }

    std::unique_ptr<Module> clone() const override {
        return std::make_unique<ReLU>(*this);
    }
};

struct Tanh : public Module {
    Tanh(std::size_t out) : Module(0, out) {}

    void initialize() override {}

    Output forward(const Input &input) override {
        for (int i = 0; i < input.size(); ++i) {
            mockOutput[i] = std::tanh(input[i]);
        }
        return mockOutput;
    }

    std::unique_ptr<Module> clone() const override {
        return std::make_unique<Tanh>(*this);
    }
};


struct Net {
    std::vector<std::unique_ptr<Module>> modules;

    Net() {}

    void initialize() {
        this->initialized = true;

        /* values.resize(modules[0]->in); */

        for (auto && mod : modules) {
            mod->initialize();
        }
    }

    Output predict(Input input) const {
        assert(input.size() == modules[0]->in && "Size of observation != net input");

        for (auto && mod : modules) {
            input = mod->forward(input);
        }

        return input;
    }

    Weights getWeights() {
        Weights allWeights;

        for (auto && mod : modules) {
            allWeights.insert(allWeights.end(), mod->weights.begin(), mod->weights.end());
        }

        return allWeights;
    }

    void loadWeights(const Weights &weights) {
        size_t beginOffset = 0;

        for (auto && mod : modules) {
            auto start = weights.begin() + beginOffset;
            auto end = weights.begin() + beginOffset + mod->weights.size();
            beginOffset += mod->weights.size();

            Weights subW(start, end);
            mod->weights = subW;
        }

    }

private: 
    bool initialized = false;
};
