#include "Mesh.hpp"
#include <mars_interfaces/graphics/GraphicsManagerInterface.h>

namespace mars
{
    namespace ode_collision
    {
        using namespace utils;
        using namespace interfaces;
        using namespace configmaps;

        Mesh::Mesh(interfaces::CollisionInterface* space, std::shared_ptr<interfaces::DynamicObject> movable, ConfigMap& config) : 
            Object(space, movable, config),
            myVertices{nullptr},
            myIndices{nullptr},
            myTriMeshData{nullptr},
            vertexcount{0},
            indexcount{0}
        {
            fprintf(stderr, "ode_collision: Mesh constructor.\n");
        }

        Mesh::~Mesh(void)
        {
            freeMemory();
        }

        void Mesh::freeMemory()
        {
            if(myVertices)
            {
                free(myVertices);
                myVertices = nullptr;
                vertexcount = 0;
            }
            if(myIndices)
            {
                free(myIndices);
                myIndices = nullptr;
                indexcount = 0;
            }
            if(myTriMeshData)
            {
                dGeomTriMeshDataDestroy(myTriMeshData);
                myTriMeshData = nullptr;
            }
        }

        Object* Mesh::instantiate(interfaces::CollisionInterface* space, std::shared_ptr<interfaces::DynamicObject> movable, ConfigMap& config)
        {
            return new Mesh{space, movable, config};
        }

        void Mesh::setMeshData(snmesh &mesh)
        {
            vertexcount = mesh.vertexcount;
            indexcount = mesh.indexcount;

            freeMemory();

            myVertices = (dVector3*)calloc(vertexcount, sizeof(dVector3));
            myIndices = (dTriIndex*)calloc(indexcount, sizeof(dTriIndex));

            // first we have to copy the mesh data to prevent errors in case
            // of double to float conversion
            for(unsigned long i=0; i<vertexcount; i++)
            {
                myVertices[i][0] = static_cast<dReal>(mesh.vertices[i][0]);
                myVertices[i][1] = static_cast<dReal>(mesh.vertices[i][1]);
                myVertices[i][2] = static_cast<dReal>(mesh.vertices[i][2]);
            }

            for(unsigned long i=0; i<indexcount; i++)
            {
                myIndices[i] = static_cast<dTriIndex>(mesh.indices[i]);
            }
        }

        // todo: add proper error handling -> setMeshData have to be called before createGeom is called...
        bool Mesh::createGeom()
        {
            assert(vertexcount > 0);

            name << config["name"];
            // // build the ode representation
            myTriMeshData = dGeomTriMeshDataCreate();
            
            dReal minx = std::numeric_limits<dReal>::max();
            dReal miny = std::numeric_limits<dReal>::max();
            dReal minz = std::numeric_limits<dReal>::max();
            dReal maxx = std::numeric_limits<dReal>::min();
            dReal maxy = std::numeric_limits<dReal>::min();
            dReal maxz = std::numeric_limits<dReal>::min();
            for(unsigned long i=0; i<vertexcount; i++)
            {
                minx = std::min(minx, myVertices[i][0]);
                miny = std::min(miny, myVertices[i][1]);
                minz = std::min(minz, myVertices[i][2]);
                maxx = std::max(maxx, myVertices[i][0]);
                maxy = std::max(maxy, myVertices[i][1]);
                maxz = std::max(maxz, myVertices[i][2]);
            }
            // rescale
            const dReal sx = static_cast<double>(config["extend"]["x"])/(maxx-minx);
            const dReal sy = static_cast<double>(config["extend"]["y"])/(maxy-miny);
            const dReal sz = static_cast<double>(config["extend"]["z"])/(maxz-minz);
            for(unsigned long i=0; i<vertexcount; i++)
            {
                myVertices[i][0] *= sx;
                myVertices[i][1] *= sy;
                myVertices[i][2] *= sz;
            }

            // TODO :what to do here. how can we calculate this??
            dGeomTriMeshDataBuildSimple(myTriMeshData, reinterpret_cast<dReal*>(myVertices), vertexcount, myIndices, indexcount) ;

            nGeom = dCreateTriMesh(space->getSpace(), myTriMeshData, 0, 0, 0);
            // we could need this in the collision callback
            dGeomSetData(nGeom, this);
            objectCreated = true;
            return true;
        }

        void Mesh::setSize(const utils::Vector &size)
        {
            const dReal sx = size.x()/static_cast<double>(config["extend"]["x"]);
            const dReal sy = size.y()/static_cast<double>(config["extend"]["y"]);
            const dReal sz = size.z()/static_cast<double>(config["extend"]["z"]);
            //LOG_ERROR("%s (%lu): %g %g %g", name.c_str(), drawID, size.x(), size.y(), size.z());

            for(unsigned long i=0; i<vertexcount; i++)
            {
                myVertices[i][0] *= sx;
                myVertices[i][1] *= sy;
                myVertices[i][2] *= sz;
            }
            dGeomTriMeshDataBuildSimple(myTriMeshData, reinterpret_cast<dReal*>(myVertices), vertexcount, myIndices, indexcount) ;
            if(graphics)
            {
                graphics->lock();
                graphics->setDrawObjectScale(drawID, Vector{sx, sy, sz});
                graphics->unlock();
            }
            // todo: how to update debug visual
        }

    } // end of namespace ode_collision
}
