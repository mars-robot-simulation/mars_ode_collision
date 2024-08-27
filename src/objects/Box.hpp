 /**
 * \file Box.hpp
 * \author Malte Langosz, Muhammad Haider Khan Lodhi
 *
 */

#pragma once

#include "Object.hpp"
#include <string>


namespace mars
{
    namespace ode_collision
    {
        class Box : public Object
        {
        public:
            Box(interfaces::CollisionInterface* space, std::shared_ptr<interfaces::DynamicObject> movable, configmaps::ConfigMap &config);
            virtual ~Box(void);
            static Object* instantiate(interfaces::CollisionInterface* space, std::shared_ptr<interfaces::DynamicObject> movable, configmaps::ConfigMap &config);
            virtual bool createGeom() override;
        };

    } // end of namespace ode_collision
} // end of namespace mars
