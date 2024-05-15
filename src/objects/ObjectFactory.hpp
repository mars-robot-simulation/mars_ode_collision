 /**
 * \file ObjectFactory.hpp
 * \author Muhammad Haider Khan Lodhi, Malte Langosz
 * \brief "ObjectFactory" implements a factory to create ode_collision objects.
 *
 */

#pragma once

#include "Object.hpp"

namespace mars
{
    namespace ode_collision
    {

        typedef Object* (*instantiateObjectfPtr)(interfaces::CollisionInterface*, std::shared_ptr<interfaces::DynamicObject>, configmaps::ConfigMap&);

        class ObjectFactory
        {
        public:
            static ObjectFactory& Instance();
            Object* createObject(const std::string &type,
                                 interfaces::CollisionInterface* space,
                                 std::shared_ptr<interfaces::DynamicObject> movable,
                                 configmaps::ConfigMap &config);
            void addObjectType(const std::string& type, instantiateObjectfPtr funcPtr);
            const std::map<const std::string, instantiateObjectfPtr>& getAvailableObjects() const noexcept
            {
                return availableObjects;
            }

        private:
            ObjectFactory();
            ~ObjectFactory();    
            std::map<const std::string, instantiateObjectfPtr> availableObjects;

        };

    }
}
