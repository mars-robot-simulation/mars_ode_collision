
 /**
 * \file Mesh.hpp
 * \author Malte Langosz
 *
 */
#pragma once
#include "Object.hpp"
#include <mars_interfaces/snmesh.h>

namespace mars
{
    namespace ode_collision
    {

        class Mesh : public Object
        {
        public:
            Mesh(interfaces::CollisionInterface* space,std::shared_ptr<interfaces::DynamicObject> movable, configmaps::ConfigMap &config);
            virtual ~Mesh(void);
            static Object* instantiate(interfaces::CollisionInterface* space,std::shared_ptr<interfaces::DynamicObject> movable, configmaps::ConfigMap &config);
            void setMeshData(interfaces::snmesh &mesh);
            virtual bool createGeom() override;
            virtual void setSize(const utils::Vector &size);

        protected:
            configmaps::ConfigMap config;
            unsigned long vertexcount, indexcount;
            dVector3 *myVertices;
            dTriIndex *myIndices;
            dTriMeshDataID myTriMeshData;
        };

    } // end of namespace ode_collision
} // end of namespace mars


