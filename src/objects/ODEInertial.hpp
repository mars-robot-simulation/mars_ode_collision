 /**
 * \file ODEInertial.hpp
 * \author Malte Langosz, Muhammad Haider Khan Lodhi
 *
 */

#pragma once

#include "ODEFrame.hpp"
#include "ODEObject.hpp"
#include <string>

//TODO remove?
#ifndef ODE11
  #define dTriIndex int
#endif

namespace mars
{
    namespace ode_physics
    {

        class ODEInertial : public ODEObject
        {
        public:
            ODEInertial(ODEFrame *frame, configmaps::ConfigMap &config);
            virtual ~ODEInertial(void);
            static ODEObject* instanciate(ODEFrame *frame, configmaps::ConfigMap &config);
            virtual bool createODEMass() override;
        private:
            configmaps::ConfigMap config;
        };

    } // end of namespace ode_physics
} // end of namespace mars
