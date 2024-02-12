// Copyright (C) 2024 ilobilo

#include <boids.hpp>

#include <glm/gtc/random.hpp>
#include <GL/gl.h>

#include <algorithm>
#include <cmath>

static void draw_box(const glm::vec3 &scale, const glm::vec3 &colour)
{
    glColor3f(colour[0], colour[1], colour[2]);
    glLineWidth(2.0);

    glBegin(GL_QUAD_STRIP);
    glNormal3f(0, 0, 1); // Face Z
    glVertex3f(-scale[0], -scale[1], scale[2]);
    glVertex3f(scale[0], -scale[1], scale[2]);
    glVertex3f(-scale[0], scale[1], scale[2]);
    glVertex3f(scale[0], scale[1], scale[2]);

    glNormal3f(0, 1, 0); // Face Y
    glVertex3f(-scale[0], scale[1], -scale[2]);
    glVertex3f(scale[0], scale[1], -scale[2]);

    glNormal3f(0, 0, -1); // Face -Z
    glVertex3f(-scale[0], -scale[1], -scale[2]);
    glVertex3f(scale[0], -scale[1], -scale[2]);

    glNormal3f(0, -1, 0); // Face -Y
    glVertex3f(-scale[0], -scale[1], scale[2]);
    glVertex3f(scale[0], -scale[1], scale[2]);
    glEnd();

    glBegin(GL_QUADS);
    glNormal3f(1, 0, 0); // Face X
    glVertex3f(scale[0], scale[1], scale[2]);
    glVertex3f(scale[0], -scale[1], scale[2]);
    glVertex3f(scale[0], -scale[1], -scale[2]);
    glVertex3f(scale[0], scale[1], -scale[2]);
    glEnd();

    glBegin(GL_QUADS);
    glNormal3f(-1, 0, 0); // Face -X
    glVertex3f(-scale[0], scale[1], scale[2]);
    glVertex3f(-scale[0], -scale[1], scale[2]);
    glVertex3f(-scale[0], -scale[1], -scale[2]);
    glVertex3f(-scale[0], scale[1], -scale[2]);
    glEnd();
}

static void draw_pyramid(const glm::vec3 &scale, const glm::vec3 &colour)
{
    glColor3f(colour[0], colour[1], colour[2]);

    glBegin(GL_TRIANGLE_FAN);
    glVertex3f(scale[0], 0, 0); // Central Vertex

    glNormal3f(1, 1, 1);
    glVertex3f(0, 0, scale[2]);
    glVertex3f(0, scale[1], 0);

    glNormal3f(1, 1, -1);
    glVertex3f(0, 0, -scale[2]);

    glNormal3f(1, -1, -1);
    glVertex3f(0, -scale[1], 0);

    glNormal3f(1, -1, 1);
    glVertex3f(0, 0, scale[2]);
    glEnd();
}

void boid::draw()
{
    const glm::vec3 speed_dir = glm::normalize(this->velocity);
    const glm::vec3 colour {
        0.5 * (1 + speed_dir.x),
        0.5 * (1 + speed_dir.y),
        0.5 * (1 + speed_dir.z)
    };

    glPushMatrix();
    {
        glTranslatef(this->position.x, this->position.y, this->position.z);

        glm::vec3 rotax = glm::cross(glm::vec3 { 1.0f, 0.0f, 0.0f }, this->velocity);
        const float angle = glm::degrees(std::atan2(glm::length(rotax), this->velocity.x));
        glRotatef(angle, rotax.x, rotax.y, rotax.z);

        draw_box({ 0.6f, 0.1f, 0.1f }, colour);
        glTranslatef(0.6f, 0.0f, 0.0f);
        draw_pyramid({ 0.4f, 0.1f, 0.1f }, { 1.0f, 1.0f, 0.0f });
    }
    glPopMatrix();
}

void boid::reset()
{
    this->neighbour_count = 0;
    this->sum_neighbour_velocity = { 0, 0, 0 };
    this->sum_neighbour_position = { 0, 0, 0 };
    this->separation_force = { 0, 0, 0 };
}

void boids::precompile_separation_forces()
{
    for (auto &iboid : this->boids_list)
    {
        for (auto &jboid : this->boids_list)
        {
            if (iboid.id == jboid.id)
                continue;

            const glm::vec3 diff = jboid.position - iboid.position;
            const float dist = glm::length(diff);

            if (dist < neighbourhood_max_dist && glm::dot(diff, iboid.velocity) >= boids::min_cos_angle * glm::length(iboid.velocity) * glm::length(diff))
            {
                iboid.neighbour_count++;
                iboid.sum_neighbour_velocity += jboid.velocity;
                iboid.sum_neighbour_position += jboid.position;
            }

            const float ths_dist = std::max(separation_min_dist, dist);
            iboid.separation_force -= separation_factor * diff / (ths_dist * ths_dist);
        }
    }
}

void boids::update_posvel(float elapsed)
{
    for (auto &boid : this->boids_list)
    {
        glm::vec3 force { 0, 0, 0 };
        if (boid.neighbour_count != 0)
        {
            auto neighbour_count = static_cast<float>(boid.neighbour_count);
            const glm::vec3 avg_pos = boid.sum_neighbour_position / neighbour_count;
            const glm::vec3 avg_vel = boid.sum_neighbour_velocity / neighbour_count;

            force += boids::cohesion_factor * (avg_pos - boid.position);
            force += boids::alignment_factor * (avg_vel - boid.velocity);
        }
        force += boid.separation_force;

        auto r = [this] { return this->dist(this->re); };
        const float dv_x = -0.5 + r();
        const float dv_y = -0.5 + r();
        const float dv_z = -0.5 + r();
        force += boids::randomness * glm::vec3 { dv_x, dv_y, dv_z };

        boid.velocity += force;
        if (glm::length(boid.velocity) > boids::max_speed)
            boid.velocity = boids::max_speed * glm::normalize(boid.velocity);

        boid.position += boid.velocity * elapsed;

        boid.reset();
    }
}

void boids::update(float elapsed)
{
    this->precompile_separation_forces();
    this->update_posvel(elapsed);

    for (auto &boid : this->boids_list)
        boid.draw();
}

void boids::add()
{
    auto r = [this] { return this->dist(this->re); };
    this->boids_list.emplace_back(glm::vec3 { r(), r(), r() }, glm::vec3 { r(), r(), r() });
}

void boids::reset()
{
    this->boids_list.clear();
    for (size_t i = 0; i < default_num_boids; i++)
        this->add();
}