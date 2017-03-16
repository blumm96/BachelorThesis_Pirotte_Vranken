//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2016, CHAI3D.
    (www.chai3d.org)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of CHAI3D nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE. 

    \author    <http://www.chai3d.org>
    \author    Chris Sewell
    \author    Charity Lu
    \author    Francois Conti
    \version   3.2.0 $Rev: 1869 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef CCollisionAABBBoxH
#define CCollisionAABBBoxH
//------------------------------------------------------------------------------
#include "graphics/CGenericArray.h"
#include "graphics/CDraw3D.h"
#include "math/CMaths.h"
#include "math/CVector3d.h"
#include <iostream>
//------------------------------------------------------------------------------
#include <vector>
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CCollisionAABBBox.h

    \brief
    Implements an axis-aligned bounding box collision tree (AABB)
*/
//==============================================================================

//==============================================================================
/*!
    \class      cCollisionAABBBox
    \ingroup    collisions

    \brief
    This structure implements the boundary nodes of an axis-aligned bounding box.

    \details
    This structure implements the data variables and methods necessary to model 
    an axis-aligned bounding box. These methods are used by the general AABB 
    collision detection algorithm (See class cCollisionAABB).
*/
//==============================================================================
struct cCollisionAABBBox
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Default constructor of cCollisionAABBBox.
<<<<<<< HEAD
    cCollisionAABBBox() { setEmpty(); };
=======
    cCollisionAABBBox() { 
		setEmpty(); 
		draw = false;
	};
>>>>>>> origin/IST

    //! Constructor of cCollisionAABBBox.
    cCollisionAABBBox(const cVector3d& a_min, 
                      const cVector3d& a_max) 
    { 
        setValue(a_min, a_max); 
    }

    //! Destructor of cCollisionAABBBox.
    virtual ~cCollisionAABBBox() {};


    //--------------------------------------------------------------------------
    // METHODS:
    //--------------------------------------------------------------------------

public:

    //--------------------------------------------------------------------------
    /*!
        \brief
        This method returns the center of the boundary box.

        \details
        This method returns the center of the boundary box.

        \return Center of boundary box.
    */
    //--------------------------------------------------------------------------
    inline cVector3d getCenter() const 
    { 
        return (m_center);
    }

<<<<<<< HEAD
=======
	/*
		\brief
		This methods returns if the box should be drawn.

		\details
		This method returns if the box should be drawn.
		This method is UHAS implemented.

		\return If the box should be drawn.
	*/
	inline bool isDraw() {
		return (draw);
	}

>>>>>>> origin/IST

    //--------------------------------------------------------------------------
    /*!
        \brief
        This method returns the extent (half the width) of the boundary box.

        \details
        This method returns the extent (half the width) of the boundary box.

        \return Extent of boundary box
    */
    //--------------------------------------------------------------------------
    inline cVector3d getExtent() const 
    { 
        return (m_extent); 
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method sets the center of the boundary box.

        \details
        This method sets the center of the boundary box.

        \param  a_center  Center of boundary box.
    */
    //--------------------------------------------------------------------------
    inline void setCenter(const cVector3d& a_center)
    { 
        m_center = a_center;
    }

<<<<<<< HEAD
=======
	/*
		\brief
		This methods sets if the box should be drawn. 

		\details
		This method sets if the box should be drawn. 
		This method is UHAS implemented.

		\param	a_draw	If the box should be drawn.
	*/
	inline void setDraw(bool a_draw) {
		draw = a_draw;
	}

>>>>>>> origin/IST

    //--------------------------------------------------------------------------
    /*!
        \brief
        This method sets the extent (half the width) of the boundary box.

        \details
        This method sets the extent (half the width) of the boundary box.

        \param  a_extent  Boundary box extent.
    */
    //--------------------------------------------------------------------------
    inline void setExtent(const cVector3d& a_extent)
    { 
        m_extent = a_extent;
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method initializes the boundary box by passing a minimum and maximum
        point.

        \details
        This method initializes the boundary box by passing a minimum and maximum
        point passed as arguments.

        \param  a_min  Minimum point of boundary box.
        \param  a_max  Maximum point of boundary box.
    */
    //--------------------------------------------------------------------------
    inline void setValue(const cVector3d& a_min, const cVector3d& a_max)
    {
        m_extent = cMul(0.5, cSub(a_max, a_min));
        m_center = cAdd(a_min, m_extent);
        m_min = a_min;
        m_max = a_max;
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method determines whether this box and an input box passed as 
        argument intersect each other.

        \details
        This method determines whether this box and an input box passed as 
        argument intersect each other.

        \param  a_box  Input box.

        \return __true__ if the input box overlaps this box, __false__ otherwise.
    */
    //--------------------------------------------------------------------------
    inline bool intersect(const cCollisionAABBBox& a_box) const
    {
        // check for overlap along each axis
        if (a_box.getLowerX() > getUpperX()) return (false);
        if (a_box.getLowerY() > getUpperY()) return (false);
        if (a_box.getLowerZ() > getUpperZ()) return (false);
        if (a_box.getUpperX() < getLowerX()) return (false);
        if (a_box.getUpperY() < getLowerY()) return (false);
        if (a_box.getUpperZ() < getLowerZ()) return (false);

        // if the boxes are not separated along any axis, a collision has occurred
        return (true);
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method determines whether the given ray intersects the boundary box.

        \details
        This method determines whether the given ray intersects the boundary box.
        Based on code by Andrew Woo from "Graphics Gems", Academic Press, 1990.

        \param  a_segmentPointA  Initial point of segment.
        \param  a_segmentPointB  End point of segment.

        \return __true__ if line segment intersects the boundary box, __false__ otherwise.
    */
    //--------------------------------------------------------------------------
    inline bool intersect(const cVector3d& a_segmentPointA, const cVector3d& a_segmentPointB) const
    {
        const int RIGHT  = 0;
        const int LEFT   = 1;
        const int MIDDLE = 2;

        double coord[3];
        char inside = true;
        char quadrant[3];
        int i;
        int whichPlane;
        double maxT[3];
        double candidatePlane[3];
        double dir[3];
        dir[0] = a_segmentPointB(0) - a_segmentPointA(0);
        dir[1] = a_segmentPointB(1) - a_segmentPointA(1);
        dir[2] = a_segmentPointB(2) - a_segmentPointA(2);

        // find candidate planes; this loop can be avoided if rays cast all from 
        // the eye (assume perspective view)
        for (i=0; i<3; i++)
        {
            if(a_segmentPointA(i) < m_min(i))
            {
                quadrant[i] = LEFT;
                candidatePlane[i] = m_min(i);
                inside = false;
            }
            else if (a_segmentPointA(i) > m_max(i))
            {
                quadrant[i] = RIGHT;
                candidatePlane[i] = m_max(i);
                inside = false;
            }
            else
            {
               quadrant[i] = MIDDLE;
            }
        }

        // ray origin inside boundary box
        if (inside)
        {
            return (true);
        }

        // calculate T distances to candidate planes
        for (i=0; i<3; i++)
        {
            if (quadrant[i] != MIDDLE && dir[i] !=0.)
                maxT[i] = (candidatePlane[i]-a_segmentPointA(i)) / dir[i];
            else
                maxT[i] = -1.;
        }

        // get largest of the maxT's for final choice of intersection
        whichPlane = 0;
        for (i=1; i<3; i++)
            if (maxT[whichPlane] < maxT[i])
                whichPlane = i;

        // check final candidate actually inside box
        if (maxT[whichPlane] < 0.) 
        {
            return (false);
        }

        for (i=0; i<3; i++)
        {
            if (whichPlane != i)
            {
                coord[i] = a_segmentPointA(i) + maxT[whichPlane] * dir[i];
                if (coord[i] < m_min(i) || coord[i] > m_max(i))
                {
                    return (false);
                }
            }
            else
            {
                coord[i] = candidatePlane[i];
            }
        }

        // ray hits box
        return (true);
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method tests whether this box contains a point passed as argument.

        \details
        This method tests whether this box contains a point \p a_point passed 
        as argument.

        \param  a_point  Point to be tested.

        \return __true__ if this box contains this point, __false__ otherwise.
    */
    //--------------------------------------------------------------------------
    inline bool contains(const cVector3d& a_point) const
    {
        // check that each of the point's coordinates are within the box's range
        if ((a_point(0) > m_min(0)) && (a_point(1) > m_min(1)) && (a_point(2) > m_min(2)) &&
            (a_point(0) < m_max(0)) && (a_point(1) < m_max(1)) && (a_point(2) < m_max(2)))
        {
            return (true);
        }
        else
        {
            return (false);
        }
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method initializes the boundary box to bound two given boundary boxes
        passed as argument.

        \details
        This method initializes the boundary box to bound two given boundary boxes
        passed as argument.

        \param  a_boxA  The first boundary box to be enclosed.
        \param  a_boxB  The second boundary box to be enclosed.
    */
    //--------------------------------------------------------------------------
    inline void enclose(const cCollisionAABBBox& a_boxA, 
                        const cCollisionAABBBox& a_boxB)
    {
        // find the minimum coordinate along each axis
        cVector3d lower(cMin(a_boxA.getLowerX(), a_boxB.getLowerX()),
                        cMin(a_boxA.getLowerY(), a_boxB.getLowerY()),
                        cMin(a_boxA.getLowerZ(), a_boxB.getLowerZ()));

        // find the maximum coordinate along each axis
        cVector3d upper(cMax(a_boxA.getUpperX(), a_boxB.getUpperX()),
                        cMax(a_boxA.getUpperY(), a_boxB.getUpperY()),
                        cMax(a_boxA.getUpperZ(), a_boxB.getUpperZ()));

        // set the center and extent of this box to enclose the two extreme points
        setValue(lower, upper);
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method resizes the boundary box to bound a new point passed as 
        argument.

        \details
        This method resizes the boundary box to bound a new point \p a_point 
        passed as argument.

        \param  a_point  Point to be enclosed.
    */
    //--------------------------------------------------------------------------
    inline void enclose (const cVector3d& a_point)
    {
        // decrease coordinates as needed to include given point
        cVector3d lower(cMin(getLowerX(), a_point(0) ),
                        cMin(getLowerY(), a_point(1) ),
                        cMin(getLowerZ(), a_point(2)));

        // increase coordinates as needed to include given point
        cVector3d upper(cMax(getUpperX(), a_point(0) ),
                        cMax(getUpperY(), a_point(1) ),
                        cMax(getUpperZ(), a_point(2)));

        // set the center and extent of this box to enclose the given point
        setValue(lower, upper);
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method resizes the boundary box to bound a new point passed as argument.

        \details
        This method resizes the boundary box to bound a new point passed as argument.

        \param  a_box  Box to be enclosed in this boundary box.
    */
    //--------------------------------------------------------------------------
    inline void enclose(const cCollisionAABBBox& a_box) 
    {
        enclose(*this, a_box); 
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method empties this boundary box.

        \details
        This method empties this boundary box.
    */
    //--------------------------------------------------------------------------
    inline void setEmpty()
    {
        const double C_INFINITY = 1.0e50;
        m_center.zero();
<<<<<<< HEAD
=======
		draw = true;
>>>>>>> origin/IST
        m_extent = cVector3d(-C_INFINITY,-C_INFINITY,-C_INFINITY);
        m_min.set( C_INFINITY, C_INFINITY, C_INFINITY);
        m_max.set(-C_INFINITY,-C_INFINITY,-C_INFINITY);
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method returns the smallest coordinate along X axis.

        \details
        This method returns the smallest coordinate along X axis.

        \return Smallest coordinate along X axis.
    */
    //--------------------------------------------------------------------------
    inline double getLowerX() const
    { 
        return (m_min(0) );
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method returns the largest coordinate along X axis.

        \details
        This method returns the largest coordinate along X axis.

        \return Largest coordinate along X axis.
    */
    //--------------------------------------------------------------------------
    inline double getUpperX() const
    { 
        return (m_max(0) ); 
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method returns the smallest coordinate along Y axis.

        \details
        This method returns the smallest coordinate along Y axis.

        \return Smallest coordinate along Y axis.
    */
    //--------------------------------------------------------------------------
    inline double getLowerY() const
    { 
        return (m_min(1) );
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method returns the largest coordinate along Y axis.

        \details
        This method returns the largest coordinate along Y axis.

        \return Largest coordinate along Y axis.
    */
    //--------------------------------------------------------------------------
    inline double getUpperY() const
    { 
        return (m_max(1) );
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This returns the smallest coordinate along Z axis.

        \details
        This returns the smallest coordinate along Z axis.

        \return Smallest coordinate along Z axis.
    */
    //--------------------------------------------------------------------------
    inline double getLowerZ() const
    { 
        return (m_min(2));
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method return the largest coordinate along Z axis.

        \details
        This method return the largest coordinate along Z axis.

        \return Largest coordinate along Z axis.
    */
    //--------------------------------------------------------------------------
    inline double getUpperZ() const
    { 
        return (m_max(2));
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method returns the index of the longest axis of the boundary box.

        \details
        This method returns the index of the longest axis of the boundary box.

        \return Index of the longest axis of the box.
    */
    //--------------------------------------------------------------------------
    inline int getLongestAxis() const
    {
        // if extent of x axis is greatest, return index 0
        if ((m_extent(0) >= m_extent(1)) && (m_extent(0) >= m_extent(2))) 
        {
            return (0);
        }
        else if ((m_extent(1) >= m_extent(0)) && (m_extent(1) >= m_extent(2))) 
        {
            return (1);
        }
        else
        {
            return (2);
        }
    }


    //--------------------------------------------------------------------------
    /*!
        \brief
        This method renders graphically the edges of the boundary box.

        \details
        This method renders graphically using OpenGL the edges of the 
        boundary box.
    */
    //--------------------------------------------------------------------------
    inline void render()
    {
<<<<<<< HEAD
        cDrawWireBox(m_min(0), m_max(0), m_min(1), m_max(1), m_min(2), m_max(2));

=======
		cDrawWireBox(m_min(0), m_max(0), m_min(1), m_max(1), m_min(2), m_max(2));
>>>>>>> origin/IST
    }

	//UHAS implement
	//print de box af
	inline void print() {
<<<<<<< HEAD
		std::cout << "minwaardes:" << getLowerX() << ", " << getLowerY() << ", " << getLowerZ() <<
			"; en maxwaardes:" << getUpperX() << ", " << getUpperY() << ", " << getUpperZ() << "; " << std::endl;
	}

	//calculate the distance between 2 boxes
	inline double distance(cCollisionAABBBox *a_box, cVector3d myLocal, cVector3d BLocal) {
=======
		std::cout << "minwaardes:" << (getLowerX()) << ", " << getLowerY() << ", " << getLowerZ() <<
			"; en maxwaardes:" << getUpperX() << ", " << getUpperY() << ", " << getUpperZ() << "; " << "en centers: " << getCenter() << std::endl;
	}

	//calculate the distance between 2 boxes
	inline double distance(cCollisionAABBBox* a_box, cVector3d myLocal, cVector3d BLocal) {
>>>>>>> origin/IST
		cVector3d* x_dist = new cVector3d();
		cVector3d* y_dist = new cVector3d();
		cVector3d* z_dist = new cVector3d();

		cVector3d* zero = new cVector3d(0.0, 0.0, 0.0);
<<<<<<< HEAD
		
		//calculate the x_difference
		if ((getUpperX()+ myLocal.x()) < (a_box->getLowerX()+BLocal.x())) x_dist->set(a_box->getLowerX()-getUpperX() + BLocal.x()- myLocal.x(), 0.0, 0.0);
		else if ((getLowerX() + myLocal.x()) > (a_box->getUpperX() + BLocal.x())) x_dist->set(-a_box->getUpperX() - BLocal.x() + myLocal.x() + getLowerX(), 0.0, 0.0);
		else x_dist->set(0.0, 0.0, 0.0);
		//std::cout << "x: " << (a_box.getLowerX() + BLocal.x()) << "---" << (getLowerX() + myLocal.x()) << std::endl;
		//calculate the y_difference
		if ((getUpperY() + myLocal.y()) < (a_box->getLowerY()+ BLocal.y())) y_dist->set(0.0, a_box->getLowerY() - getUpperY() -myLocal.y() + BLocal.y(), 0.0);
		else if ((getLowerY() + myLocal.y()) > (a_box->getUpperY()+ BLocal.y())) y_dist->set(0.0, -a_box->getUpperY() + getLowerY() + myLocal.y() - BLocal.y(), 0.0);
		else y_dist->set(0.0, 0.0, 0.0);
		//calculate the z_difference
		if ((getUpperZ()+ myLocal.z()) < (a_box->getLowerZ() + BLocal.z())) z_dist->set(0.0, 0.0, a_box->getLowerZ() - getUpperZ() - myLocal.z() + BLocal.z());
		else if ((getLowerZ() + myLocal.z()) > (a_box->getUpperZ()+BLocal.z())) z_dist->set(0.0, 0.0, -a_box->getUpperZ() + getLowerZ() + myLocal.z() - BLocal.z());
		else z_dist->set(0.0, 0.0, 0.0);

=======

		cVector3d positie1 =  cVector3d();
		cVector3d positie2 = cVector3d();
		positie1 = myLocal;
		positie2 = BLocal;
		
		//calculate the x_difference
		if ((getUpperX()+ positie1.x()) < (a_box->getLowerX()+ positie2.x())) x_dist->set(a_box->getLowerX()-getUpperX()+ positie2.x()- positie1.x(), 0.0, 0.0);
		else if ((getLowerX() + positie1.x()) > (a_box->getUpperX() + positie2.x())) x_dist->set(-a_box->getUpperX() - positie2.x() + positie1.x() + getLowerX(), 0.0, 0.0);
		else x_dist->set(0.0, 0.0, 0.0);

		//std::cout << "y: " << (a_box->getLowerY() + positie2.y()) << "---" << (getLowerY() + positie1.y()) << std::endl;
		//calculate the y_difference

		if ((getUpperY() + positie1.y()) < (a_box->getLowerY()+ positie2.y())) y_dist->set(0.0, a_box->getLowerY() - getUpperY() - positie1.y() + positie2.y(), 0.0);
		else if ((getLowerY() + positie1.y()) > (a_box->getUpperY()+ positie2.y())) y_dist->set(0.0, -a_box->getUpperY() + getLowerY() + positie1.y() - positie2.y(), 0.0);
		else y_dist->set(0.0, 0.0, 0.0);

		//calculate the z_difference
		if ((getUpperZ()+ positie1.z()) < (a_box->getLowerZ() + positie2.z())) z_dist->set(0.0, 0.0, a_box->getLowerZ() - getUpperZ() - positie1.z() + positie2.z());
		else if ((getLowerZ() + positie1.z()) > (a_box->getUpperZ()+ positie2.z())) z_dist->set(0.0, 0.0, -a_box->getUpperZ() + getLowerZ() + positie1.z() - positie2.z());
		else z_dist->set(0.0, 0.0, 0.0);

		//if ((getUpperX()) < (a_box.getLowerX())) x_dist->set(a_box.getLowerX() - getUpperX(), 0.0, 0.0);
		//else if ((getLowerX()) > (a_box.getUpperX())) x_dist->set(-a_box.getUpperX() + getLowerX(), 0.0, 0.0);
		//else x_dist->set(0.0, 0.0, 0.0);

		////std::cout << "x: " << (a_box.getLowerY() + BLocal.y()) << "---" << (getLowerY() + myLocal.y()) << std::endl;
		////calculate the y_difference

		//if ((getUpperY()) < (a_box.getLowerY())) y_dist->set(0.0, a_box.getLowerY() - getUpperY(), 0.0);
		//else if ((getLowerY()) > (a_box.getUpperY())) y_dist->set(0.0, -a_box.getUpperY() + getLowerY(), 0.0);
		//else y_dist->set(0.0, 0.0, 0.0);

		////calculate the z_difference
		//if ((getUpperZ()) < (a_box.getLowerZ())) z_dist->set(0.0, 0.0, a_box.getLowerZ() - getUpperZ());
		//else if ((getLowerZ()) > (a_box.getUpperZ())) z_dist->set(0.0, 0.0, -a_box.getUpperZ() + getLowerZ());
		//else z_dist->set(0.0, 0.0, 0.0);


>>>>>>> origin/IST
		//Make the sum of these vectors = total distance vector
		cVector3d *min_dist = new cVector3d(0.0,0.0,0.0);
		min_dist->add(*x_dist);
		min_dist->add(*y_dist);
		min_dist->add(*z_dist);

<<<<<<< HEAD
		double test = (a_box->getLowerX() - getUpperX() + BLocal.x() - myLocal.x());

=======
>>>>>>> origin/IST
		//delete all dynamic reserved mem
		delete(x_dist);
		delete(y_dist);
		delete(z_dist);

		//calculate the norm of the distance vector
		double mindist = min_dist->length();
		delete(min_dist);
		delete(zero);

		//return the mindist between the 2 boxes
<<<<<<< HEAD
		//std::cout << "mindist: " << mindist << std::endl;
=======
>>>>>>> origin/IST
		return mindist;
		//return 1;
	}

    //--------------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //--------------------------------------------------------------------------

public:

    //! The center of the boundary box.
    cVector3d m_center;

<<<<<<< HEAD
=======
	//The box only needs to be drawn when it hits another one.
	bool draw;

>>>>>>> origin/IST
    //! The extent (half the width) of the boundary box.
    cVector3d m_extent;

    //! The minimum point (along each axis) of the boundary box.
    cVector3d m_min;

    //! The maximum point (along each axis) of the boundary box.
    cVector3d m_max;
};


//------------------------------------------------------------------------------
} // namespace chai3d
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------

