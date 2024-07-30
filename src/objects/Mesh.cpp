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
            Object(space, movable),
            config{config},
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
            if(myVertices)
            {
                free(myVertices);
            }
            if(myIndices)
            {
                free(myIndices);
            }
            if(myTriMeshData)
            {
                dGeomTriMeshDataDestroy(myTriMeshData);
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
            name << config["name"];
            // // build the ode representation
            myTriMeshData = dGeomTriMeshDataCreate();
            
            dReal minx, miny, minz, maxx, maxy, maxz;
            for(unsigned long i=0; i<vertexcount; i++)
            {
                if(i==0)
                {
                    minx = myVertices[i][0];
                    maxx = myVertices[i][0];
                    miny = myVertices[i][1];
                    maxy = myVertices[i][1];
                    minz = myVertices[i][2];
                    maxz = myVertices[i][2];
                } else
                {
                    if(minx > myVertices[i][0]) minx = myVertices[i][0];
                    if(maxx < myVertices[i][0]) maxx = myVertices[i][0];
                    if(miny > myVertices[i][1]) miny = myVertices[i][1];
                    if(maxy < myVertices[i][1]) maxy = myVertices[i][1];
                    if(minz > myVertices[i][2]) minz = myVertices[i][2];
                    if(maxz < myVertices[i][2]) maxz = myVertices[i][2];
                }
            }
            // rescale
            dReal sx = static_cast<double>(config["extend"]["x"])/(maxx-minx);
            dReal sy = static_cast<double>(config["extend"]["y"])/(maxy-miny);
            dReal sz = static_cast<double>(config["extend"]["z"])/(maxz-minz);
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
            dReal sx = static_cast<double>(config["extend"]["x"]);
            dReal sy = static_cast<double>(config["extend"]["y"]);
            dReal sz = static_cast<double>(config["extend"]["z"]);
            //LOG_ERROR("%s (%lu): %g %g %g", name.c_str(), drawID, size.x(), size.y(), size.z());
            sx = size.x()/sx;
            sy = size.y()/sy;
            sz = size.z()/sz;

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
                graphics->setDrawObjectScale(drawID, Vector(sx, sy, sz));
                graphics->unlock();
            }
            // todo: how to update debug visual
        }

    } // end of namespace ode_collision
}
