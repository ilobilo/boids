// Copyright (C) 2024 ilobilo

#include <trackball.hpp>

#include <glm/ext.hpp>
#include <GL/gl.h>

#include <algorithm>
#include <cmath>

namespace trackball
{
    axes camera::compute() const
    {
        const auto front = glm::normalize(glm::vec3 { -std::cos(this->phi) * std::sin(this->theta), std::sin(this->phi), -std::cos(this->phi) * std::cos(this->theta) });
        const auto right = glm::normalize(glm::cross(front, camera::gravity_up));
        const auto down = glm::normalize(glm::cross(front, right));

        return { right, down, front };
    }

    void camera::lookat()
    {
        auto axes = this->compute();

        const auto eye = this->centre - (this->rad * axes.front);
        glm::mat4 view = glm::lookAt(eye, this->centre, camera::gravity_up);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glLoadMatrixf(glm::value_ptr(view));
    }

    void camera::rotate(float dx, float dy)
    {
        this->phi += 0.5 * M_PI * dy;
        this->theta += 0.5 * M_PI * dx;

        static constexpr float PHI_MIN = -0.5 * M_PI + 0.001f;
        static constexpr float PHI_MAX = 0.5 * M_PI - 0.001f;

        this->phi = std::clamp(this->phi, PHI_MIN, PHI_MAX);
    }

    void camera::zoom(long num)
    {
        this->rad *= std::pow(1.1f, num);
    }

    void camera::shift_centre(float dx, float dy)
    {
        auto axes = this->compute();
        this->centre -= this->rad * dx * axes.right + this->rad * dy * axes.down;
    }
} // trackball
