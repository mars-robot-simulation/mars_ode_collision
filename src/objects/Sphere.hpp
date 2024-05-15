 /**
 * \file Sphere.hpp
 * \author Malte Langosz, Muhammad Haider Khan Lodhi
 *
 */

#pragma once
#include <mars_utils/MutexLocker.h>
#include "Object.hpp"
#include <string>

namespace mars
{
    namespace ode_collision
    {

        class Sphere : public Object
        {
        public:
            Sphere(interfaces::CollisionInterface* space, std::shared_ptr<interfaces::DynamicObject> movable, configmaps::ConfigMap &config);
            virtual ~Sphere(void);
            static Object* instantiate(interfaces::CollisionInterface* space, std::shared_ptr<interfaces::DynamicObject> movable, configmaps::ConfigMap &config);
            virtual bool createGeom() override;
        private:
            configmaps::ConfigMap config;
        };

    } // end of namespace ode_collision
} // end of namespace mars
