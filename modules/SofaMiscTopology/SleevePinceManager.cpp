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
#include <SofaMiscTopology/SleevePinceManager.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/ObjectFactory.h>

#include <sofa/core/visual/VisualParams.h>
#include <sofa/simulation/Node.h>
#include <sofa/core/objectmodel/DataFileName.h>

#include <sofa/core/objectmodel/KeypressedEvent.h>
using sofa::core::objectmodel::KeypressedEvent;

#include <sofa/core/objectmodel/KeyreleasedEvent.h>
using sofa::core::objectmodel::KeyreleasedEvent;

#include <sofa/simulation/Simulation.h>


#include <time.h>

#ifndef NDEBUG
    #define DEBUG_MSG true
#else
    #define DEBUG_MSG false
#endif

namespace sofa
{

namespace component
{

namespace misc
{

SOFA_DECL_CLASS(SleevePinceManager)

using namespace defaulttype;

typedef sofa::core::behavior::MechanicalState< sofa::defaulttype::Vec3Types > mechaState;

int SleevePinceManagerClass = core::RegisterObject("Handle sleeve Pince.")
        .add< SleevePinceManager >();


SleevePinceManager::SleevePinceManager()
    : m_pathMord1(initData(&m_pathMord1, "pathMord1", "Path to mord1"))
    , m_pathMord2(initData(&m_pathMord2, "pathMord2", "Path to mord2"))
    , m_pathModel(initData(&m_pathModel, "pathModel", "Path to model"))
    , m_mord1(NULL)
    , m_mord2(NULL)
    , m_model(NULL)
{
    this->f_listening.setValue(true);
}


SleevePinceManager::~SleevePinceManager()
{

}


void SleevePinceManager::init()
{
    const std::string& pathMord1 = m_pathMord1.getValue();
    const std::string& pathMord2 = m_pathMord2.getValue();
    const std::string& pathModel = m_pathModel.getValue();

    if (pathMord1.empty() && pathMord2.empty())
    {
        serr << "no input mords found !!" << sendl;
        return;
    }

    this->getContext()->get(m_mord1, pathMord1);
    this->getContext()->get(m_mord2, pathMord2);
    this->getContext()->get(m_model, pathModel);

    if (m_mord1 == NULL || m_mord2 == NULL || m_model == NULL)
    {
        serr << "error mechanical state not found" << sendl;
        return;
    }

    std::cout << "m_mord1: " << m_mord1->getName() << std::endl;
    std::cout << "m_mord2: " << m_mord2->getName() << std::endl;
    std::cout << "m_mord2: " << m_model->getName() << std::endl;

    computeBoundingBox();
}

int SleevePinceManager::testModels()
{
    if (m_mord1 == NULL)
        return -20;

    if (m_mord2 == NULL)
        return -21;

    if (m_model == NULL)
        return -22;

    return 52;
}

bool SleevePinceManager::computeBoundingBox()
{
    if (m_mord1 == NULL || m_mord2 == NULL)
    {
        std::cout << "error mechanical state not found" << std::endl;
        const std::string& pathMord1 = m_pathMord1.getValue();
        const std::string& pathMord2 = m_pathMord2.getValue();
        this->getContext()->get(m_mord1, pathMord1);
        this->getContext()->get(m_mord2, pathMord2);

        if (m_mord1 == NULL || m_mord2 == NULL)
            return false;
    }

    for (int i = 0; i < 3; ++i)
    {
        m_min[i] = 10000;
        m_max[i] = -10000;
    }


    for (int i = 0; i < m_mord1->getSize(); i++)
    {
        SReal x = m_mord1->getPX(i);
        SReal y = m_mord1->getPY(i);
        SReal z = m_mord1->getPZ(i);
        std::cout << "drawLine: " << x << " " << y << " " << z << std::endl;
        if (x < m_min[0])
            m_min[0] = x;
        if (y < m_min[1])
            m_min[1] = y;
        if (z < m_min[2])
            m_min[2] = z;

        if (x > m_max[0])
            m_max[0] = x;
        if (y > m_max[1])
            m_max[1] = y;
        if (z > m_max[2])
            m_max[2] = z;
    }

    for (int i = 0; i < m_mord2->getSize(); i++)
    {
        SReal x = m_mord2->getPX(i);
        SReal y = m_mord2->getPY(i);
        SReal z = m_mord2->getPZ(i);

        if (x < m_min[0])
            m_min[0] = x;
        if (y < m_min[1])
            m_min[1] = y;
        if (z < m_min[2])
            m_min[2] = z;

        if (x > m_max[0])
            m_max[0] = x;
        if (y > m_max[1])
            m_max[1] = y;
        if (z > m_max[2])
            m_max[2] = z;
    }

    return true;
}

void SleevePinceManager::reinit()
{
/*    if (!m_useDataInputs.getValue())
        this->readDataFile();
        */
}

std::vector< int > SleevePinceManager::grabModel()
{
    std::vector< int > candidatID;

    if (m_model == NULL)
        return candidatID;
    
    std::cout << "m_model->getSize(): " << m_model->getSize() << std::endl;
    for (int i = 0; i < m_model->getSize(); i++)
    {
        SReal x = m_model->getPX(i);
        SReal y = m_model->getPY(i);
        SReal z = m_model->getPZ(i);
        if (x > m_min[0] && x < m_max[0] 
            && y > m_min[1] && y < m_max[1] 
            && z > m_min[2] && z < m_max[2] )
        {
            candidatID.push_back(i);
            std::cout << "Add: " << i << std::endl;
        }
    }
    
    return candidatID;
}



void SleevePinceManager::handleEvent(sofa::core::objectmodel::Event* event)
{
    if (KeypressedEvent::checkEventType(event))
    {
        KeypressedEvent *ev = static_cast<KeypressedEvent *>(event);

        switch (ev->getKey())
        {

        case 'T':
        case 't':
        {
            computeBoundingBox();
            grabModel();
            break;
        }
        }
    }
}

void SleevePinceManager::draw(const core::visual::VisualParams* vparams)
{
    if (!vparams->displayFlags().getShowBehaviorModels())
        return;

    vparams->drawTool()->drawLine(m_min, m_max, Vec<4, float>(1.0, 0.0, 1.0, 1.0));
   // std::cout << "drawLine: " << m_min[0] << " " << m_min[1] << " " << m_min[2] << std::endl;
}



} // namespace misc

} // namespace component

} // namespace sofa
