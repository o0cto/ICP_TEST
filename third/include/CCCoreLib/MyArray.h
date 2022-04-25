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

#ifndef MY_ARRAY_HEADER
#define MY_ARRAY_HEADER

//CCCoreLib
#include "CCShareable.h"

//System
#include <vector>


namespace MyScan
{
	//! Shareable array that can be properly inserted in the DB tree
	template <class Type, int N> class MyArray : public std::vector<Type>, public CCShareable
	{
	public:

		//! Base type
		typedef MyArray<Type, N> Base;

		//! Default constructor
		MyArray()
		{
			// setFlagState(CC_LOCKED, true);
		}

		//! Duplicates array
		virtual Base* clone()
		{
			Base* cloneArray = new Base();
			if (!copy(*cloneArray))
			{
				//error message already issued
				cloneArray->release();
				cloneArray = nullptr;
			}
			return cloneArray;
		}

		//! Copies the content of this array in another one
		bool copy(Base& dest) const
		{
			try
			{
				//copy only the data
				static_cast<std::vector<Type>&>(dest) = static_cast<const std::vector<Type>&>(*this);
			}
			catch (const std::bad_alloc&)
			{
				// ccLog::Warning("[MyArray::copy] Not enough memory");
				return false;
			}
			return true;
		}

		//! Reserves memory (no exception thrown)
		bool reserveSafe(size_t count)
		{
			try
			{
				this->reserve(count);
			}
			catch (const std::bad_alloc&)
			{
				//not enough memory
				return false;
			}
			return true;
		}

		//! Returns whether some memory has been allocated or not
		inline bool isAllocated() const { return this->capacity() != 0; }
		
		//! Resizes memory (no exception thrown)
		bool resizeSafe(size_t count, bool initNewElements = false, const Type* valueForNewElements = nullptr)
		{
			try
			{
				if (initNewElements)
				{
					if (!valueForNewElements)
					{
						// ccLog::Warning("[MyArray::resizeSafe] Internal error: no new element specified");
						return false;
					}
					this->resize(count, *valueForNewElements);
				}
				else
				{
					this->resize(count);
				}
			}
			catch (const std::bad_alloc&)
			{
				//not enough memory
				return false;
			}
			return true;
		}

		//Shortcuts (for backward compatibility)
		inline Type& getValue(size_t index) { return this->at(index); }
		inline const Type& getValue(size_t index) const { return this->at(index); }
		inline void setValue(size_t index, const Type& value) { this->at(index) = value; }
		inline void addElement(const Type& value) { this->emplace_back(value); }
		inline void fill(const Type& value) { if (this->empty()) this->resize(this->capacity(), value); else std::fill(this->begin(), this->end(), value); }
		inline unsigned currentSize() const { return static_cast<unsigned>(this->size()); }
		inline void clear(bool releaseMemory = false) { if (releaseMemory) this->resize(0); else this->std::vector<Type>::clear(); }
		inline void swap(size_t i1, size_t i2) { std::swap(this->at(i1), this->at(i2)); }

	protected:

		//! Destructor (protected)
		/** Use release instead.
		**/
		virtual ~MyArray() {}

	};
}


#endif //CC_ARRAY_HEADER
