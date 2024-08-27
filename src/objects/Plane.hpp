
 /**
 * \file Plane.hpp
 * \author Malte Langosz
 */

#pragma once

#include <mars_utils/MutexLocker.h>
#include "Object.hpp"
#include <string>

namespace mars
{
    namespace ode_collision
    {

        class Plane : public Object
        {
        public:
            Plane(interfaces::CollisionInterface* space, std::shared_ptr<interfaces::DynamicObject> movable, configmaps::ConfigMap &config);
            virtual ~Plane(void);
            static Object* instantiate(interfaces::CollisionInterface* space, std::shared_ptr<interfaces::DynamicObject> movable, configmaps::ConfigMap &config);
            virtual bool createGeom() override;
            virtual void updateTransform(void) override;
        };

    } // end of namespace ode_collision
} // end of namespace mars

