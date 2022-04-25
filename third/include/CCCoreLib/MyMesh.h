//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#ifndef MY_SCAN_MY_MESH_HEADER
#define MY_SCAN_MY_MESH_HEADER

//CCCoreLib
#include "PointProjectionTools.h"
#include "SimpleTriangle.h"
#include "GenericIndexedMesh.h"
#include "BoundingBox.h"

//Local
#include "MyPointCloud.h"
#include "MyArray.h"

class MyPointCloud;

namespace CCCoreLib
{
	class GenericProgressCallback;
	class ReferenceCloud;
}

namespace MyScan
{
    //! Triangular mesh
    class  __attribute__((visibility("default"))) MyMesh : public CCCoreLib::GenericIndexedMesh
    {
    public:

        //! Default MyMesh constructor
        /** \param vertices the vertices cloud
            \param uniqueID unique ID (handle with care)
        **/
        // MyMesh();
        MyMesh()
        {
            m_associatedCloud = nullptr;
            m_triVertIndexes = nullptr;
            m_globalIterator = 0;
        }

        MyMesh(MyPointCloud* vertices);

        //! MyMesh constructor (from a CCCoreLib::GenericIndexedMesh)
        /** The GenericIndexedMesh should refer to a known ccGenericPointCloud.
            \param giMesh the GenericIndexedMesh
            \param giVertices giMesh vertices
        **/
        MyMesh(CCCoreLib::GenericIndexedMesh* giMesh, MyPointCloud* giVertices);

        //! Default destructor
        ~MyMesh() override;

        //! Sets the associated vertices cloud (warning)
        void setAssociatedCloud(MyPointCloud* cloud);     

        //inherited methods (ccGenericMesh) 去掉重载！！！！
        inline MyPointCloud* getAssociatedCloud() const 
        { return m_associatedCloud; }
        void refreshBB();
        void computeInterpolationWeights(unsigned triIndex, const CCVector3& P, CCVector3d& weights) const;//去掉重载
        unsigned capacity() const;

        //inherited methods (GenericIndexedMesh)
        void forEach(genericTriangleAction action) override;
        void placeIteratorAtBeginning() override;
        CCCoreLib::GenericTriangle* _getNextTriangle() override; //temporary
        CCCoreLib::GenericTriangle* _getTriangle(unsigned triangleIndex) override; //temporary
        CCCoreLib::VerticesIndexes* getNextTriangleVertIndexes() override;
        CCCoreLib::VerticesIndexes* getTriangleVertIndexes(unsigned triangleIndex) override;
        void getTriangleVertices(unsigned triangleIndex, CCVector3& A, CCVector3& B, CCVector3& C) const override;
        unsigned size() const override;
        void getBoundingBox(CCVector3& bbMin, CCVector3& bbMax) override;

        //const version of getTriangleVertIndexes
        const virtual CCCoreLib::VerticesIndexes* getTriangleVertIndexes(unsigned triangleIndex) const;

        //! Shifts all triangles indexes
        /** \param shift index shift (positive)
        **/
        void shiftTriangleIndexes(unsigned shift);

        //! Flips the triangle
        /** Swaps the second and third vertices indexes
        **/
        void flipTriangles();

        //! Adds a triangle to the mesh
        /** \warning Bounding-box validity is broken after a call to this method.
            However, for the sake of performance, no call to notifyGeometryUpdate
            is made automatically. Make sure to do so when all modifications are done!
            \param i1 first vertex index (relatively to the vertex cloud)
            \param i2 second vertex index (relatively to the vertex cloud)
            \param i3 third vertex index (relatively to the vertex cloud)
        **/
        void addTriangle(unsigned i1, unsigned i2, unsigned i3);

        //! Reserves the memory to store the vertex indexes (3 per triangle)
        /** \param n the number of triangles to reserve
            \return true if the method succeeds, false otherwise
        **/
        bool reserve(size_t n);

        //! Resizes the array of vertex indexes (3 per triangle)
        /** If the new number of elements is smaller than the actual size,
            the overflooding elements will be deleted.
            \param n the new number of triangles
            \return true if the method succeeds, false otherwise
        **/
        bool resize(size_t n);

        //! Removes unused capacity
        inline void shrinkToFit() { if (size() < capacity()) resize(size()); }

        bool reservePerTriangleMtlIndexes();

        //! Removes any per-triangle material indexes
        void removePerTriangleMtlIndexes();

        //! Adds triangle material index for next triangle
        /** Cf. ccMesh::reservePerTriangleMtlIndexes.
            \param mtlIndex triangle material index
        **/
        void addTriangleMtlIndex(int mtlIndex);

        //! Mesh scalar field processes
        enum MESH_SCALAR_FIELD_PROCESS {	SMOOTH_MESH_SF,		/**< Smooth **/
                                            ENHANCE_MESH_SF,	/**< Enhance **/
        };

        //! Applies process to the mesh scalar field (the one associated to its vertices in fact)
        /** A very simple smoothing/enhancement algorithm based on
            each vertex direct neighbours. Prior to calling this method,
            one should check first that the vertices are associated to a
            scalar field.
            Warning: the processed scalar field must be enabled for both
            INPUT & OUTPUT! (see ccGenericCloud::setCurrentScalarField)
            \param process either 'smooth' or 'enhance'
        **/
        bool processScalarField(MESH_SCALAR_FIELD_PROCESS process);

        //! Swaps two triangles
        /** Automatically updates internal structures (i.e. lookup tables for
            material, normals, etc.).
        **/
        void swapTriangles(unsigned index1, unsigned index2);

        //! Default octree level for the 'mergeDuplicatedVertices' algorithm
        static const unsigned char DefaultMergeDuplicateVerticesLevel = 10;

        //! Merges duplicated vertices
        // bool mergeDuplicatedVertices(unsigned char octreeLevel = DefaultMergeDuplicateVerticesLevel, QWidget* parentWidget = nullptr);
        
    public: //from ccGenericMesh 
        //! Samples points on a mesh
        MyPointCloud* samplePoints(	bool densityBased,
                                    double samplingParameter,
                                    bool withNormals,
                                    bool withRGB,
                                    bool withTexture,
                                    CCCoreLib::GenericProgressCallback* pDlg = nullptr);
        //! Imports the parameters from another mesh
        /** Only the specific parameters are imported.
        **/
        void importParametersFrom(const MyMesh* mesh);

        //! Computes the point that corresponds to the given uv (barycentric) coordinates
        bool computePointPosition(unsigned triIndex, const CCVector2d& uv, CCVector3& P, bool warningIfOutside = true) const;


    protected: //methods

        //! Same as other 'computeInterpolationWeights' method with a set of 3 vertices indexes
        void computeInterpolationWeights(const CCCoreLib::VerticesIndexes& vertIndexes, const CCVector3& P, CCVector3d& weights) const;

    protected: //members

        //! associated cloud (vertices)
        MyPointCloud* m_associatedCloud;

        //! Container of per-triangle vertices indexes (3)
        using triangleIndexesContainer = MyArray<CCCoreLib::VerticesIndexes, 3>;
        //! Triangles' vertices indexes (3 per triangle)
        triangleIndexesContainer* m_triVertIndexes;

        //! Iterator on the list of triangles
        unsigned m_globalIterator;
        //! Dump triangle structure to transmit temporary data
        CCCoreLib::SimpleRefTriangle m_currentTriangle;

        //! Bounding-box
        BoundingBox m_bBox;

        //! Set of triplets of indexes referring to mesh texture coordinates
        using triangleTexCoordIndexesSet = MyArray<Tuple3i, 3>;

        //! Set of triplets of indexes referring to mesh normals
        using triangleNormalsIndexesSet = MyArray<Tuple3i, 3>;
        //! Mesh normals indexes (per-triangle)
        // triangleNormalsIndexesSet* m_triNormalIndexes;
    };
}

#endif //CC_MESH_HEADER
