#include "Interfaces/Logger.h"
#include "Interfaces/EnvironmentObject.h"
#include "FlatEarth.h"

#define FLAT_EARTH_ENV_CREATED "The environment FlatEarth has been created."
#define FLAT_EARTH_ENV_DESTROYED "The environment FlatEarth has been destroyed."

FlatEarth::FlatEarth(Logger& logger) {
    this->logger = &logger;
    this->logger->debug_log(FLAT_EARTH_ENV_CREATED);
}

FlatEarth::~FlatEarth() {
    this->logger->debug_log(FLAT_EARTH_ENV_DESTROYED);
}

void FlatEarth::update(double dt) {
    for (auto&& o : this->env_objects)
        o->update(dt);
}

std::string FlatEarth::str() {
    return std::string(
        "<FlatEarth:Environment>"
    );
}