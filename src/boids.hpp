// Copyright (C) 2024 ilobilo

#pragma once

#include <glm/glm.hpp>
#include <random>
#include <vector>

static constexpr size_t default_num_boids = 50;

class boid
{
    friend struct boids;

    static inline size_t next_id = 0;

    private:
    size_t neighbour_count;
    glm::vec3 sum_neighbour_velocity;
    glm::vec3 sum_neighbour_position;
    glm::vec3 separation_force;

    public:
    const size_t id;

    glm::vec3 position;
    glm::vec3 velocity;

    void draw();
    void reset();

    boid(glm::vec3 position, glm::vec3 velocity) :
        neighbour_count { 0 }, sum_neighbour_velocity { 0, 0, 0 },
        sum_neighbour_position { 0, 0, 0 }, separation_force { 0, 0, 0 },
        id(next_id++), position(position), velocity(velocity) { }
};

struct boids
{
    static inline float neighbourhood_max_dist = 10;
    static inline float separation_min_dist = 1;

    static inline float separation_factor = 0.02;
    static inline float cohesion_factor = 0.07;
    static inline float alignment_factor = 0.005;

    static inline float max_speed = 10;
    static inline float min_cos_angle = -0.8f;
    static inline float randomness = 0;

    std::vector<boid> boids_list;

    std::random_device rd;
    std::mt19937_64 re;
    std::uniform_real_distribution<> dist;

    void precompile_separation_forces();
    void update_posvel(float elapsed);

    void update(float elapsed);
    void add();
    void reset();

    boids() : boids_list { }, rd(), re(rd()), dist(0, 1) { this->reset(); }
};
