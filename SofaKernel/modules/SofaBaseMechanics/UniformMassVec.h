/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, development version     *
*                (c) 2006-2016 INRIA, USTL, UJF, CNRS, MGH                    *
*                                                                             *
* This library is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This library is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this library; if not, write to the Free Software Foundation,     *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.          *
*******************************************************************************
*                               SOFA :: Modules                               *
*                                                                             *
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SOFA_COMPONENT_MASS_UNIFORMMASSVEC_H
#define SOFA_COMPONENT_MASS_UNIFORMMASSVEC_H
#include "config.h"

#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/topology/Topology.h>
#include <sofa/core/objectmodel/Context.h>
#include <sofa/helper/accessor.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/DataTypeInfo.h>
#include <SofaBaseMechanics/AddMToMatrixFunctor.h>
#include <sofa/simulation/Simulation.h>
#include <iostream>
#include <string.h>
#include <SofaBaseMechanics/UniformMass.h>

namespace sofa
{

namespace component
{

namespace mass
{

using helper::WriteAccessor;
using helper::ReadAccessor;
using helper::WriteOnlyAccessor;
using helper::vector;

using std::list;

using core::behavior::Mass;
using core::topology::BaseMeshTopology;
using core::topology::TopologyChange;
using core::MechanicalParams;
using core::behavior::MultiMatrixAccessor;
using core::visual::VisualParams;
using core::ConstVecCoordId;

using defaulttype::BaseVector;
using defaulttype::Vec;
using defaulttype::Vec3d;
using defaulttype::DataTypeInfo;
using defaulttype::BaseMatrix;
using defaulttype::Vector6 ;

using sofa::component::mass::UniformMass ;

struct Enabled {} ;
struct Disabled {} ;

template <class DataTypes, class TMassType>
class UniformMassVec : public UniformMass<DataTypes, TMassType>
{
public:
    typedef UniformMassVec<DataTypes, TMassType> ThisClass ;

    SOFA_CLASS(SOFA_TEMPLATE2(UniformMassVec, DataTypes, TMassType),
               SOFA_TEMPLATE2(UniformMass, DataTypes, TMassType));

    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Deriv Deriv;
    typedef typename Coord::value_type Real;
    typedef core::objectmodel::Data<VecCoord> DataVecCoord;
    typedef core::objectmodel::Data<VecDeriv> DataVecDeriv;

    ////////////////////////// Inherited attributes ////////////////////////////
    /// https://gcc.gnu.org/onlinedocs/gcc/Name-lookup.html
    /// Bring inherited attributes and function in the current lookup context.
    /// otherwise any access to the base::attribute would require
    /// the "this->" approach.
    using core::behavior::ForceField<DataTypes>::mstate ;
    using core::objectmodel::BaseObject::getContext;

    using UniformMass<DataTypes, TMassType>::setMass ;
    using UniformMass<DataTypes, TMassType>::getMass ;

    using UniformMass<DataTypes, TMassType>::m_doesTopoChangeAffect;
    using UniformMass<DataTypes, TMassType>::d_mass ;                 ///< the mass of each particle
    using UniformMass<DataTypes, TMassType>::d_totalMass;    ///< if >0 : total mass of this body

    using UniformMass<DataTypes, TMassType>::d_showCenterOfGravity; /// to display the center of gravity of the system
    using UniformMass<DataTypes, TMassType>::d_showAxisSize;        /// to display the center of gravity of the system

    using UniformMass<DataTypes, TMassType>::d_computeMappingInertia;
    using UniformMass<DataTypes, TMassType>::d_showInitialCenterOfGravity;

    using UniformMass<DataTypes, TMassType>::d_showX0; /// to display the rest positions

    using UniformMass<DataTypes, TMassType>::d_localRange;
    using UniformMass<DataTypes, TMassType>::d_indices;

    using UniformMass<DataTypes, TMassType>::d_handleTopoChange;
    using UniformMass<DataTypes, TMassType>::d_preserveTotalMass;
    ////////////////////////////////////////////////////////////////////////////

    virtual void reinit() ;

    Vector6 getMomentum ( const MechanicalParams*,
                          const DataVecCoord& d_x,
                          const DataVecDeriv& d_v ) const ;


    template<class T>
    Vector6 dirtyHackGetMomentum ( const MechanicalParams*,
                                   const DataVecCoord& d_x,
                                   const DataVecDeriv& d_v,
                                   typename std::enable_if<T::spatial_dimensions==3, int>::type t) const ;

    template<class T>
    Vector6 dirtyHackGetMomentum ( const MechanicalParams*,
                                   const DataVecCoord& d_x,
                                   const DataVecDeriv& d_v,
                                   typename std::enable_if<T::spatial_dimensions!=3, int>::type t) const ;


    void draw(const VisualParams* vparams);

    void addMDxToVector(defaulttype::BaseVector *resVect,
                        const VecDeriv* dx,
                        SReal mFact,
                        unsigned int& offset) ;

    SReal getPotentialEnergy ( const MechanicalParams* params,
                               const DataVecCoord& d_x  ) const ;


protected:
    UniformMassVec() ;
    ~UniformMassVec() ;

public:
    static std::string className(const ThisClass*t) {
        SOFA_UNUSED(t) ;
        return "UniformMass";
    }

    static std::string templateName(const ThisClass*t) {
        return Inherit1::templateName(t) ;
    }
};

} // namespace mass

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_UNIFORMMASSVEC_H
