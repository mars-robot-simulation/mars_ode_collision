
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
            Mesh(interfaces::CollisionInterface* space, std::shared_ptr<interfaces::DynamicObject> movable, configmaps::ConfigMap& config);
            virtual ~Mesh(void);
            static Object* instantiate(interfaces::CollisionInterface* space, std::shared_ptr<interfaces::DynamicObject> movable, configmaps::ConfigMap& config);
            void setMeshData(interfaces::snmesh& mesh);
            virtual bool createGeom() override;
            virtual void setSize(const utils::Vector& size);

        protected:
            unsigned long vertexcount;
            unsigned long indexcount;
            dVector3* myVertices;
            dTriIndex* myIndices;
            dTriMeshDataID myTriMeshData;
        
        private:
            void freeMemory();
        };

    } // end of namespace ode_collision
} // end of namespace mars


