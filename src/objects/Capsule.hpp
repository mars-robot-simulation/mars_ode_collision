
/**
 * \file Capsule.hpp
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

        class Capsule : public Object
        {
        public:
            Capsule(interfaces::CollisionInterface* space, std::shared_ptr<interfaces::DynamicObject> movable, configmaps::ConfigMap &config);
            virtual ~Capsule(void);
            static Object *instantiate(interfaces::CollisionInterface *space,std::shared_ptr<interfaces::DynamicObject> movable, configmaps::ConfigMap &config);
            virtual bool createGeom() override;
        };

    } // end of namespace ode_collision
} // end of namespace mars
