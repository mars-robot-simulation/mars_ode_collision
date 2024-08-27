 /**
 * \file Heightfield.hpp
 * \author Malte Langosz, Muhammad Haider Khan Lodhi
 *
 */

#pragma once

#include "Object.hpp"
#include <mars_interfaces/terrainStruct.h>

namespace mars
{
    namespace ode_collision
    {

        class Heightfield : public Object
        {
        public:
            Heightfield(interfaces::CollisionInterface* space, std::shared_ptr<interfaces::DynamicObject> movable, configmaps::ConfigMap& config);
            virtual ~Heightfield(void);
            static Object* instantiate(interfaces::CollisionInterface* space, std::shared_ptr<interfaces::DynamicObject> movable, configmaps::ConfigMap& config);
            virtual bool createGeom() override;
            dReal heightCallback(int x, int y);
            void setTerrainStruct(interfaces::terrainStruct* t);
            virtual void updateTransform(void) override;
            //override due to orientation offset
            void getRotation(utils::Quaternion* q) const;

        protected:
            interfaces::terrainStruct* terrain;
            dReal* height_data;
      
        };

    } // end of namespace ode_collision
} // end of namespace mars
