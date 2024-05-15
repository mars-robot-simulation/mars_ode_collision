/**
 * \file ODECylinder.hpp
 * \author Malte Langosz
 *
 */

#pragma once

#include "Object.hpp"
#include <string>

namespace mars
{
    namespace ode_collision
    {

        class Cylinder : public Object
        {
        public:
            Cylinder(interfaces::CollisionInterface* space, std::shared_ptr<interfaces::DynamicObject> movable, configmaps::ConfigMap &config);
            virtual ~Cylinder(void);
            static Object *instantiate(interfaces::CollisionInterface *space,std::shared_ptr<interfaces::DynamicObject> movable, configmaps::ConfigMap &config);
            virtual bool createGeom() override;

        private:
            configmaps::ConfigMap config;
        };

    } // end of namespace ode_collision
} // end of namespace mars
