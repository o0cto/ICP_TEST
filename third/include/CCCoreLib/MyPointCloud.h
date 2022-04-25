// SPDX-License-Identifier: LGPL-2.0-or-later
// Copyright © EDF R&D / TELECOM ParisTech (ENST-TSI)

#pragma once

#ifdef _MSC_VER
//To get rid of the warnings about dominant inheritance
#pragma warning( disable: 4250 )
#endif

//CCCoreLib
#include "BoundingBox.h"
#include "GenericIndexedCloudPersist.h"
#include "ScalarField.h"
#include "DgmOctree.h"

//STL
#include <vector>

class MyMesh;

// extern const unsigned CC_MAX_NUMBER_OF_POINTS_PER_CLOUD;

using namespace CCCoreLib;
namespace CCCoreLib
{
	class GenericProgressCallback;
	class ReferenceCloud;
}

namespace MyScan
{
	//! A storage-efficient point cloud structure that can also handle an unlimited number of scalar fields
    class __attribute__((visibility("default")))   MyPointCloud : public CCCoreLib::GenericIndexedCloudPersist
	{
		// friend class ccMesh;
		friend class MyMesh;

	public:
		//! Default constructor
		  MyPointCloud()
		{
			m_currentPointIndex = 0;
			m_currentInScalarFieldIndex = -1;
			m_currentOutScalarFieldIndex = -1;
			m_pointSize = 0;
		}

		//! Copy Constructor
		  MyPointCloud(const MyPointCloud &rhs)
		{
			m_points = rhs.m_points;
			m_bbox = rhs.m_bbox;
			m_currentPointIndex = rhs.m_currentPointIndex;
			m_scalarFields = rhs.m_scalarFields;
			m_currentInScalarFieldIndex = rhs.m_currentInScalarFieldIndex;
			m_currentOutScalarFieldIndex = rhs.m_currentOutScalarFieldIndex;
			m_pointsVisibility = rhs.m_pointsVisibility;
			m_pointSize = rhs.m_pointSize;
			// Link all the existing scalar fields so they don't get deleted when rhs goes out of scope
			for (ScalarField *sf : m_scalarFields)
			{
				sf->link();
			}
		}

		//! Default destructor
		virtual ~  MyPointCloud()
		{
			deleteAllScalarFields_tpl();
			unallocateVisibilityArray();
			deleteOctree();
			clear();
		}

        // //! Returns class ID
        // //! ccGenericPointCloud --> ccShiftedObject --> ccHObject
    	// CC_CLASS_ENUM getClassID() const override { return CC_TYPES::POINT_CLOUD; }

		//! Copy Assignment
		MyPointCloud &operator=(const   MyPointCloud &rhs)
		{
			m_points = rhs.m_points;
			m_bbox = rhs.m_bbox;
			m_currentPointIndex = rhs.m_currentPointIndex;
			m_scalarFields = rhs.m_scalarFields;
			m_currentInScalarFieldIndex = rhs.m_currentInScalarFieldIndex;
			m_currentOutScalarFieldIndex = rhs.m_currentOutScalarFieldIndex;

			// Link all the existing scalar fields so they don't get deleted when rhs goes out of scope
			for (ScalarField *sf : m_scalarFields)
			{
				sf->link();
			}

			return *this;
		}
        const MyPointCloud& operator +=(MyPointCloud*);

		inline unsigned size() const override { return static_cast<unsigned>(m_points.size()); }

		void forEach(GenericCloud::genericPointAction action) override
		{
			//there's no point of calling forEach if there's no activated scalar field!
			ScalarField* currentOutScalarFieldArray = getCurrentOutScalarField();
			if (!currentOutScalarFieldArray)
			{
				assert(false);
				return;
			}

			unsigned n = size();
			for (unsigned i = 0; i < n; ++i)
			{
				action(m_points[i], (*currentOutScalarFieldArray)[i]);
			}
		}

		void getBoundingBox(CCVector3& bbMin, CCVector3& bbMax) override
		{
			if (!m_bbox.isValid())
			{
				m_bbox.clear();
				for (const CCVector3& P : m_points)
				{
					m_bbox.add(P);
				}
			}

			bbMin = m_bbox.minCorner();
			bbMax = m_bbox.maxCorner();
		}

		void placeIteratorAtBeginning() override { m_currentPointIndex = 0; }

		const CCVector3* getNextPoint() override { return (m_currentPointIndex < m_points.size() ? point(m_currentPointIndex++) : 0); }

		bool enableScalarField() override
		{
			if (m_points.empty() && m_points.capacity() == 0)
			{
				//on must call resize or reserve on the cloud first
				return false;
			}

			ScalarField* currentInScalarField = getCurrentInScalarField();

			if (!currentInScalarField)
			{
				//if we get there, it means that either the caller has forgot to create
				//(and assign) a scalar field to the cloud, or that we are in a compatibility
				//mode with old/basic behaviour: a unique SF for everything (input/output)

				//we look for any already existing "default" scalar field
				m_currentInScalarFieldIndex = getScalarFieldIndexByName("Default");
				if (m_currentInScalarFieldIndex < 0)
				{
					//if not, we create it
					m_currentInScalarFieldIndex = addScalarField_tpl("Default");
					if (m_currentInScalarFieldIndex < 0) //Something went wrong
					{
						return false;
					}
				}

				currentInScalarField = getCurrentInScalarField();
				assert(currentInScalarField);
			}

			//if there's no output scalar field either, we set this new scalar field as output also
			if (!getCurrentOutScalarField())
			{
				m_currentOutScalarFieldIndex = m_currentInScalarFieldIndex;
			}

			if (m_points.empty())
			{
				//if the cloud is empty, with a reserved capacity, we do the same on the SF
				return currentInScalarField->reserveSafe(m_points.capacity());
			}
			else
			{
				//otherwise we resize the SF with the current number of points so that they match
				return currentInScalarField->resizeSafe(m_points.size());
			}
		}

		bool isScalarFieldEnabled() const override
		{
			ScalarField* currentInScalarFieldArray = getCurrentInScalarField();
			if (!currentInScalarFieldArray)
			{
				return false;
			}

			std::size_t sfValuesCount = currentInScalarFieldArray->size();
			return (sfValuesCount != 0 && sfValuesCount >= m_points.size());
		}

		void setPointScalarValue(unsigned pointIndex, ScalarType value) override
		{
			assert(m_currentInScalarFieldIndex >= 0 && m_currentInScalarFieldIndex < static_cast<int>(m_scalarFields.size()));
			//slow version
			//ScalarField* currentInScalarFieldArray = getCurrentInScalarField();
			//if (currentInScalarFieldArray)
			//	currentInScalarFieldArray->setValue(pointIndex,value);

			//fast version
			m_scalarFields[m_currentInScalarFieldIndex]->setValue(pointIndex, value);
		}

		ScalarType getPointScalarValue(unsigned pointIndex) const override
		{
			assert(m_currentOutScalarFieldIndex >= 0 && m_currentOutScalarFieldIndex < static_cast<int>(m_scalarFields.size()));

			return m_scalarFields[m_currentOutScalarFieldIndex]->getValue(pointIndex);
		}

		inline const CCVector3* getPoint(unsigned index) const override { return point(index); }
		inline void getPoint(unsigned index, CCVector3& P) const override { P = *point(index); }

		inline const CCVector3* getPointPersistentPtr(unsigned index) const override { return point(index); }

		//! Adds a scalar values to the active 'in' scalar field
		/** \param value a scalar value
		**/
		void addPointScalarValue(ScalarType value)
		{
			assert(m_currentInScalarFieldIndex >= 0 && m_currentInScalarFieldIndex < static_cast<int>(m_scalarFields.size()));

			//fast version
			m_scalarFields[m_currentInScalarFieldIndex]->addElement(value);
		}

        //! Resizes all the active features arrays
    	/** This method is meant to be called after having increased the cloud
    		population (if the final number of inserted point is lower than the
    		reserved size). Otherwise, it fills all new elements with blank values.
    		\return true if ok, false if there's not enough memory
    	**/


		//! Resizes the point database
		/** The cloud database is resized with the specified size. If the new size
			is smaller, the overflooding points will be deleted. If its greater,
			the database is filled with blank points (warning, the
			PointCloud::addPoint method will insert points after those ones).
			\param newNumberOfPoints the new number of points
			\return true if the method succeeds, false otherwise
		**/
		bool resize_tpl(unsigned newCount)
		{
			std::size_t oldCount = m_points.size();

			//we try to enlarge the 3D points array
			try
			{
				m_points.resize(newCount);
			}
			catch (const std::bad_alloc&)
			{
				return false;
			}

			//then the scalar fields
			for (std::size_t i = 0; i < m_scalarFields.size(); ++i)
			{
				if (!m_scalarFields[i]->resizeSafe(newCount))
				{
					//if something fails, we restore the previous size for already processed SFs!
					for (std::size_t j = 0; j < i; ++j)
					{
						m_scalarFields[j]->resize(oldCount);
						m_scalarFields[j]->computeMinAndMax();
					}
					//we can assume that newCount > oldNumberOfPoints, so it should always be ok
					m_points.resize(oldCount);
					return false;
				}
				m_scalarFields[i]->computeMinAndMax();
			}

			return true;
		}

		//! Clears the cloud database
		/** Equivalent to resize(0).
		**/
		void reset()
		{
			m_points.resize(0);
			deleteAllScalarFields_tpl();
			placeIteratorAtBeginning();
			invalidateBoundingBox_tpl();
		}

		//! Adds a 3D point to the database
		/** To ensure the best efficiency, the database memory must have already
			been reserved (with PointCloud::reserve). Otherwise nothing happens.
			\param P a 3D point
		**/
		void addPoint(const CCVector3 &P)
		{
			//NaN coordinates check
			if (	P.x != P.x
				||	P.y != P.y
				||	P.z != P.z)
			{
				//replace NaN point by (0, 0, 0)
				CCVector3 fakeP(0, 0, 0);
				m_points.push_back(fakeP);
			}
			else
			{
				m_points.push_back(P);
			}

			m_bbox.setValidity(false);
		}

		//! Invalidates bounding box
		/** Bounding box will be recomputed next time a request is made to 'getBoundingBox'.
		**/
		void invalidateBoundingBox_tpl() { m_bbox.setValidity(false); }

		/*** scalar fields management ***/

		//! Returns the number of associated (and active) scalar fields
		/** \return the number of active scalar fields
		**/
		inline unsigned getNumberOfScalarFields() const { return static_cast<unsigned>(m_scalarFields.size()); }

		//! Returns a pointer to a specific scalar field
		/** \param index a scalar field index
			\return a pointer to a ScalarField structure, or 0 if the index is invalid.
		**/
		ScalarField* getScalarField(int index) const
		{
			return (index >= 0 && index < static_cast<int>(m_scalarFields.size()) ? m_scalarFields[index] : 0);
		}

		//! Returns the name of a specific scalar field
		/** \param index a scalar field index
			\return a pointer to a string structure (null-terminated array of characters), or 0 if the index is invalid.
		**/
		const char* getScalarFieldName(int index) const
		{
			return (index >= 0 && index < static_cast<int>(m_scalarFields.size()) ? m_scalarFields[index]->getName() : 0);
		}

		//! Returns the index of a scalar field represented by its name
		/** \param name a scalar field name
			\return an index (-1 if the scalar field couldn't be found)
		**/
		int getScalarFieldIndexByName(const char* name) const
		{
			std::size_t sfCount = m_scalarFields.size();
			for (std::size_t i = 0; i < sfCount; ++i)
			{
				//we don't accept two SF with the same name!
				if (strcmp(m_scalarFields[i]->getName(), name) == 0)
					return static_cast<int>(i);
			}

			return -1;
		}

		//! Returns the scalar field currently associated to the cloud input
		/** See PointCloud::setPointScalarValue.
			\return a pointer to the currently defined INPUT scalar field (or 0 if none)
		**/
		inline ScalarField* getCurrentInScalarField() const { return getScalarField(m_currentInScalarFieldIndex); }

		//! Returns the scalar field currently associated to the cloud output
		/** See PointCloud::getPointScalarValue.
			\return a pointer to the currently defined OUTPUT scalar field (or 0 if none)
		**/
		inline ScalarField* getCurrentOutScalarField() const { return getScalarField(m_currentOutScalarFieldIndex); }

		//! Sets the INPUT scalar field
		/** This scalar field will be used by the PointCloud::setPointScalarValue method.
			\param index a scalar field index (or -1 if none)
		**/
		inline void setCurrentInScalarField(int index) { m_currentInScalarFieldIndex = index; }

		//! Returns current INPUT scalar field index (or -1 if none)
		inline int getCurrentInScalarFieldIndex() { return m_currentInScalarFieldIndex; }

		//! Sets the OUTPUT scalar field
		/** This scalar field will be used by the PointCloud::getPointScalarValue method.
			\param index a scalar field index (or -1 if none)
		**/
		inline void setCurrentOutScalarField(int index) { m_currentOutScalarFieldIndex = index; }

		//! Returns current OUTPUT scalar field index (or -1 if none)
		inline int getCurrentOutScalarFieldIndex() { return m_currentOutScalarFieldIndex; }

		//! Sets both the INPUT & OUTPUT scalar field
		/** This scalar field will be used by both PointCloud::getPointScalarValue
			and PointCloud::setPointScalarValue methods.
			\param index a scalar field index
		**/
		inline void setCurrentScalarField(int index) { setCurrentInScalarField(index); setCurrentOutScalarField(index); }

		//! Creates a new scalar field and registers it
		/** Warnings:
			- the name must be unique (the method will fail if a SF with the same name already exists)
			- this method DOES resize the scalar field to match the current cloud size
			\param uniqueName scalar field name (must be unique)
			\return index of this new scalar field (or -1 if an error occurred)
		**/
		int addScalarField_tpl(const char* uniqueName)
		{
			//we don't accept two SF with the same name!
			if (getScalarFieldIndexByName(uniqueName) >= 0)
			{
				return -1;
			}

			//create requested scalar field
			ScalarField* sf = new ScalarField(uniqueName);
			if (!sf || (size() && !sf->resizeSafe(m_points.size())))
			{
				//Not enough memory!
				if (sf)
					sf->release();
				return -1;
			}

			try
			{
				//we don't want 'm_scalarFields' to grow by 50% each time! (default behavior of std::vector::push_back)
				m_scalarFields.resize(m_scalarFields.size() + 1, sf);
			}
			catch (const std::bad_alloc&) //out of memory
			{
				sf->release();
				return -1;
			}

			// Link the scalar field to this cloud
			sf->link();

			return static_cast<int>(m_scalarFields.size()) - 1;
		}

		//! Renames a specific scalar field
		/** Warning: name must not be already given to another SF!
			\param index scalar field index
			\param newName new name
			\return success
		**/
		bool renameScalarField(int index, const char* newName)
		{
			if (getScalarFieldIndexByName(newName) < 0)
			{
				ScalarField* sf = getScalarField(index);
				if (sf)
				{
					sf->setName(newName);
					return true;
				}
			}
			return false;
		}

		//! Deletes a specific scalar field
		/** WARNING: this operation may modify the scalar fields order
			(especially if the deleted SF is not the last one). However
			current IN & OUT scalar fields will stay up-to-date (while
			their index may change).
			\param index index of scalar field to be deleted
		**/
		void deleteScalarField_tpl(int index)
		{
			int sfCount = static_cast<int>(m_scalarFields.size());
			if (index < 0 || index >= sfCount)
				return;

			//we update SF roles if they point to the deleted scalar field
			if (index == m_currentInScalarFieldIndex)
				m_currentInScalarFieldIndex = -1;
			if (index == m_currentOutScalarFieldIndex)
				m_currentOutScalarFieldIndex = -1;

			//if the deleted SF is not the last one, we swap it with the last element
			int lastIndex = sfCount - 1; //lastIndex>=0
			if (index < lastIndex) //i.e.lastIndex>0
			{
				std::swap(m_scalarFields[index], m_scalarFields[lastIndex]);
				//don't forget to update SF roles also if they point to the last element
				if (lastIndex == m_currentInScalarFieldIndex)
					m_currentInScalarFieldIndex = index;
				if (lastIndex == m_currentOutScalarFieldIndex)
					m_currentOutScalarFieldIndex = index;
			}

			//we can always delete the last element (and the vector stays consistent)
			m_scalarFields.back()->release();
			m_scalarFields.pop_back();
		}

		// ! Deletes all scalar fields associated to this cloud
		void deleteAllScalarFields_tpl()
		{
			m_currentInScalarFieldIndex = m_currentOutScalarFieldIndex = -1;

			while (!m_scalarFields.empty())
			{
				m_scalarFields.back()->release();
				m_scalarFields.pop_back();
			}
		}

		//! Returns cloud capacity (i.e. reserved size)
		inline unsigned capacity() const { return static_cast<unsigned>(m_points.capacity()); }


	public: //clone, copy, etc. 由 ccPointCloud 实现
        //! [ccGenericPointCloud]
        // ccGenericPointCloud* clone(ccGenericPointCloud* destCloud = nullptr, bool ignoreChildren = false);// 去掉重载
        void clear();// 去掉重载


    public: //features deletion/clearing
        //! Erases the cloud points
        /** Prefer MyPointCloud::clear by default.
            \warning DANGEROUS
        **/
        void unallocatePoints();
    public: //features allocation/resize
        
        //! Reserves memory to store the points coordinates
        /** Before adding points to the cloud (with addPoint())
            be sure to reserve the necessary amount of memory
            with this method. If the number of new elements is
            smaller than the actual one, nothing will happen.
            \param _numberOfPoints number of points to reserve the memory for
            \return true if ok, false if there's not enough memory
        **/
        bool reserveThePointsTable(unsigned _numberOfPoints);
        //! Removes unused capacity
        inline void shrinkToFit() { if (size() < capacity()) resize(size()); }

        CCVector3 computeGravityCenter();

    public:
        bool resize(unsigned numberOfPoints);
		//! Reserves memory for the point database
		/** This method tries to reserve some memory to store points
			that will be inserted later (with PointCloud::addPoint).
			If the new number of points is smaller than the actual one,
			nothing happens.
			\param newNumberOfPoints the new number of points
			\return true if the method succeeds, false otherwise
		**/
		bool reserve(unsigned newCapacity);
        bool reserveApi(unsigned numberOfPoints);
		void hello();
        //scalar-fields management
        //inherited from base class
        void deleteScalarField(int index);
        void deleteAllScalarFields(); 
        int addScalarField(const char* uniqueName);
        int addScalarField(ScalarField* sf);

        //inherited from base class
        void invalidateBoundingBox();

        void swapPoints(unsigned firstIndex, unsigned secondIndex);
        //! Translates cloud
        /** \param T translation vector
        **/
        void translate(const CCVector3& T);

	protected:
		//! Swaps two points (and their associated scalar values!)
		void swapPoints_tpl(unsigned firstIndex, unsigned secondIndex)
		{
			if (	firstIndex == secondIndex
				||	firstIndex >= m_points.size()
				||	secondIndex >= m_points.size())
			{
				return;
			}

			std::swap(m_points[firstIndex], m_points[secondIndex]);

			for (std::size_t i = 0; i < m_scalarFields.size(); ++i)
			{
				m_scalarFields[i]->swap(firstIndex, secondIndex);
			}
		}

		//! Returns non const access to a given point
		/** WARNING: index must be valid
			\param index point index
			\return pointer on point stored data
		**/
		inline CCVector3* point(unsigned index) { assert(index < size()); return &(m_points[index]); }

		//! Returns const access to a given point
		/** WARNING: index must be valid
			\param index point index
			\return pointer on point stored data
		**/
		inline const CCVector3* point(unsigned index) const { assert(index < size()); return &(m_points[index]); }

		//! 3D Points database
		std::vector<CCVector3> m_points;

		//! Bounding-box
		BoundingBox m_bbox;

		//! 'Iterator' on the points db
		unsigned m_currentPointIndex;

		//! Associated scalar fields
		std::vector<ScalarField*> m_scalarFields;

		//! Index of current scalar field used for input
		int m_currentInScalarFieldIndex;

		//! Index of current scalar field used for output
		int m_currentOutScalarFieldIndex;
	
	public:
		//! Sets point size
		/** Overrides default value one if superior than 0
			(see glPointSize).
		**/
		void setPointSize(unsigned size = 0) { m_pointSize = static_cast<unsigned char>(size); }

		//! Returns current point size
		/** 0 means that the cloud will use current OpenGL value
			(see glPointSize).
		**/
		unsigned char getPointSize() const { return m_pointSize; }

		//! Imports the parameters from another cloud
		/** Only the specific parameters are imported.
		**/
		// void importParametersFrom(const ccGenericPointCloud* cloud);

		//! Point picking (brute force or octree-driven)
		/** \warning the octree-driven method only works if pickWidth == pickHeight
		**/
		// bool pointPicking(	const CCVector2d& clickPos,
		// 					const ccGLCameraParameters& camera,
		// 					int& nearestPointIndex,
		// 					double& nearestSquareDist,
		// 					double pickWidth = 2.0,
		// 					double pickHeight = 2.0,
		// 					bool autoComputeOctree = false);

		/***************************************************
					Octree management,
					由ccGenericPointCloud 实现
		***************************************************/
		//! Computes the cloud octree
		/** The octree bounding-box is automatically defined as the smallest
			3D cube that totally encloses the cloud.
			\warning any previously attached octree will be deleted,
					even if the new octree computation failed.
			\param progressCb the caller can get some notification of the process progress through this callback mechanism (see CCCoreLib documentation)
			\param autoAddChild whether to automatically add the computed octree as child of this cloud or not
			\return the computed octree
		**/
		virtual  DgmOctree* computeOctree(CCCoreLib::GenericProgressCallback* progressCb = nullptr, bool autoAddChild = true);
	
		//! Returns the associated octree (if any)
		virtual  DgmOctree* getOctree() const;
		//! Sets the associated octree
		virtual void setOctree( DgmOctree* octree, bool autoAddChild = true);
		//! Returns the associated octree proxy (if any)
		// virtual ccOctreeProxy* getOctreeProxy() const;

		//! Erases the octree
		virtual void deleteOctree();

		/***************************************************
					Visibility array
                由ccGenericPointCloud 实现
	***************************************************/
		//! Array of "visibility" information for each point
		/** See <CCConst.h>
		**/
		using VisibilityTableType = std::vector<unsigned char>;
		
		//! Returns associated visibility array
		virtual inline VisibilityTableType& getTheVisibilityArray() { return m_pointsVisibility; }

		//! Returns associated visibility array (const version)
		virtual inline const VisibilityTableType& getTheVisibilityArray() const { return m_pointsVisibility; }

		//! Returns a ReferenceCloud equivalent to the visibility array
		/** \param visTable visibility table (optional, otherwise the cloud's default one will be used)
			\param silent don't issue warnings if no visible point is present
			\return the visible points as a ReferenceCloud
		**/
		// virtual CCCoreLib::ReferenceCloud* getTheVisiblePoints(const VisibilityTableType* visTable = nullptr, bool silent = false) const;
		
		//! Returns whether the visibility array is allocated or not
		virtual bool isVisibilityTableInstantiated() const;

		//! Resets the associated visibility array
		/** Warning: allocates the array if it was not done yet!
		**/
		virtual bool resetVisibilityArray();

		//! Inverts the visibility array
		virtual void invertVisibilityArray();

		//! Erases the points visibility information
		virtual void unallocateVisibilityArray();		

	protected:
		//! Per-point visibility table
		/** If this table is allocated, only values set to POINT_VISIBLE
			will be considered as visible/selected.
		**/
		VisibilityTableType m_pointsVisibility;
		//! Point size (won't be applied if 0)
		unsigned char m_pointSize;
	};

}
