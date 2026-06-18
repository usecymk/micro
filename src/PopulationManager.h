#ifndef POPULATION_MANAGER_H
#define POPULATION_MANAGER_H

#include <vector>
#include <memory>
#include <raylib.h>
#include <raymath.h>
#include "Bacteria.h"

class PopulationManager {
public:
    int maxPopulation = 100;
    float reproductionRadius = 1.5f;

    // Call this once per frame in your main loop
    void update(std::vector<std::unique_ptr<Bacteria>>& population, float dt) {
        if (population.size() >= (size_t)maxPopulation) return;

        for (size_t i = 0; i < population.size(); i++) {
            for (size_t j = i + 1; j < population.size(); j++) {
                
                float dist = Vector3Distance(population[i]->getPosition(), population[j]->getPosition());
                
                // 1. Proximity Check
                if (dist < reproductionRadius) {
                    
                    // 2. "Needs Met" Check (Hunger is a proxy for energy here)
                    if (population[i]->bsm.state.hunger < 20.0f && 
                        population[j]->bsm.state.hunger < 20.0f) {
                        
                        // 3. Increment Maturity
                        population[i]->scale += dt * 0.05f; 
                        
                        // 4. Trigger Reproduction
                        if (population[i]->scale >= 1.5f) {
                            spawnNewBacteria(population, *population[i], *population[j]);
                            
                            // Reset maturity and apply reproduction cost
                            population[i]->scale = 1.0f;
                            population[i]->bsm.state.hunger += 40.0f;
                            population[j]->bsm.state.hunger += 40.0f;
                            
                            // Stop after one successful pair per frame to prevent runaway growth
                            return; 
                        }
                    }
                }
            }
        }
    }

private:
    void spawnNewBacteria(std::vector<std::unique_ptr<Bacteria>>& population, 
                          Bacteria& pA, Bacteria& pB) {
        
        Vector3 midPos = Vector3Lerp(pA.getPosition(), pB.getPosition(), 0.5f);
        population.push_back(std::make_unique<Bacteria>(midPos));
    }
};

#endif