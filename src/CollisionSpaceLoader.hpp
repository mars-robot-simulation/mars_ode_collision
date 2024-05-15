/**
 * \file CollisionSpaceLoader.hpp
 * \author Malte Langosz
 * \brief "CollisionSpaceLoader" factory class to generate new physics instances.
 *
 */

#pragma once

#include "CollisionSpace.hpp"

#include <lib_manager/LibInterface.hpp>

namespace mars
{
    namespace ode_collision
    {

        class CollisionSpaceLoader : public lib_manager::LibInterface
        {
        public:
            explicit CollisionSpaceLoader(lib_manager::LibManager *theManager);
            virtual ~CollisionSpaceLoader(void);

            // LibInterface methods
            int getLibVersion() const
            {
                return 1;
            }

            const std::string getLibName() const
            {
                return std::string("mars_ode_collision");
            }


            CREATE_MODULE_INFO();

            std::shared_ptr<interfaces::CollisionInterface> createCollisionSpace(interfaces::ControlCenter *control=nullptr);
        };

    } // end of namespace ode_collision
} // end of namespace mars
