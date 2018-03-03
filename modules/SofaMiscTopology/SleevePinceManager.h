/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, development version     *
*                (c) 2006-2017 INRIA, USTL, UJF, CNRS, MGH                    *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SOFA_COMPONENT_MISC_SleevePinceManager_H
#define SOFA_COMPONENT_MISC_SleevePinceManager_H
#include "config.h"

#include <sofa/core/topology/BaseMeshTopology.h>
#include <sofa/core/topology/BaseTopology.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/objectmodel/Event.h>

#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>

#include <SofaDeformable/StiffSpringForceField.h>

#include <sofa/defaulttype/DataTypeInfo.h>
#include <sofa/simulation/Visitor.h>

#include <SofaBaseTopology/TriangleSetGeometryAlgorithms.h>
#include <SofaGeneralObjectInteraction/AttachConstraint.h>
#include <sofa/defaulttype/Vec.h>

#ifdef SOFA_HAVE_ZLIB
#include <zlib.h>
#endif

#include <fstream>

namespace sofa
{

namespace component
{

namespace misc
{

#ifdef SOFA_FLOAT
typedef float Real; ///< alias
#else
typedef double Real; ///< alias
#endif

typedef sofa::component::interactionforcefield::StiffSpringForceField< sofa::defaulttype::Vec3Types > StiffSpringFF;
typedef sofa::component::interactionforcefield::StiffSpringForceField< sofa::defaulttype::Vec3Types > StiffSpringFF;
typedef sofa::component::projectiveconstraintset::AttachConstraint< sofa::defaulttype::Vec3Types > AttachConstraint;

/** Read file containing topological modification. Or apply input modifications
 * A timestep has to be established for each modification.
 *
 * SIMPLE METHODE FOR THE MOMENT. DON'T HANDLE MULTIPLE TOPOLOGIES
*/
class SOFA_MISC_TOPOLOGY_API SleevePinceManager: public core::objectmodel::BaseObject
{
public:
    SOFA_CLASS(SleevePinceManager,core::objectmodel::BaseObject);



protected:
    SleevePinceManager();

    virtual ~SleevePinceManager();

  

public:
    virtual void init() override;

    virtual void reinit() override;

    const sofa::helper::vector< int >& grabModel();
    const sofa::helper::vector< int >& getGrabedIds() { return m_idgrabed; }
    void releaseGrab();

    void createFF();

    virtual void handleEvent(sofa::core::objectmodel::Event* event) override;


    /// Pre-construction check method called by ObjectFactory.
    /// Check that DataTypes matches the MeshTopology.
    template<class T>
    static bool canCreate(T*& obj, core::objectmodel::BaseContext* context, core::objectmodel::BaseObjectDescription* arg)
    {
      /*  if (context->getMeshTopology() == NULL)
            return false;*/

        return BaseObject::canCreate(obj, context, arg);
    }

    void draw(const core::visual::VisualParams* vparams) override;

    int testModels();

    Data<std::string> m_pathMord1;
    Data<std::string> m_pathMord2;
    Data<std::string> m_pathModel;

    bool computeBoundingBox();
    sofa::defaulttype::Vector3 m_min, m_max;

    sofa::helper::vector <int> m_idgrabed;

public:
    sofa::core::behavior::BaseMechanicalState* m_mord1;
    sofa::core::behavior::BaseMechanicalState* m_mord2;
    sofa::core::behavior::BaseMechanicalState* m_model;

    StiffSpringFF::SPtr m_forcefieldUP;
    StiffSpringFF::SPtr m_forcefieldDOWN;

    AttachConstraint::SPtr m_attach;

};


} // namespace misc

} // namespace component

} // namespace sofa

#endif
