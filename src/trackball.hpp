// Copyright (C) 2024 ilobilo

#pragma once

#include <glm/glm.hpp>

namespace trackball
{
    inline constexpr float fovy = 60.0f;
    inline constexpr float nearclip = 0.1f;
    inline constexpr float farclip = 150.0f;
    inline constexpr size_t fps = 60;

    struct axes
    {
        glm::vec3 right;
        glm::vec3 down;
        glm::vec3 front;
    };

    struct camera
    {
        static constexpr glm::vec3 gravity_up { 0.f, -1.f, 0.f };

        private:
        glm::vec3 centre;
        float rad;
        float phi;
        float theta;

        axes compute() const;

        public:
        void lookat();
        void rotate(float dx, float dy);
        void zoom(long num);
        void shift_centre(float dx, float dy);

        void reset(const glm::vec3 &centre, float rad, float phi = 0.0f, float theta = 0.0f)
        {
            this->centre = centre;
            this->rad = rad;
            this->phi = phi;
            this->theta = theta;
        }

        void recentre(const glm::vec3 &centre)
        {
            this->centre = centre;
        }

        camera(const glm::vec3 &centre, float rad, float phi = 0.0f, float theta = 0.0f) :
            centre(centre), rad(rad), phi(phi), theta(theta) { }
    };
} // trackball
