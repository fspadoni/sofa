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
    , m_forcefieldUP(NULL)
    , m_forcefieldDOWN(NULL)
{
    this->f_listening.setValue(true);
    m_idgrabed.clear();
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
        //std::cout << "drawLine: " << x << " " << y << " " << z << std::endl;
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

const sofa::helper::vector< int >& SleevePinceManager::grabModel()
{
    m_idgrabed.clear();

    if (m_model == NULL)
        return m_idgrabed;
    
    sofa::helper::vector<int> idsModel;

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
            idsModel.push_back(i);
            
        }
    }

    std::cout << "Broad Phase detection: " << idsModel.size() << std::endl;

    if (idsModel.size() == 0)
        return m_idgrabed;


    // Second distance to spheres

    StiffSpringFF* stiffspringforcefield_UP = static_cast<StiffSpringFF*>(m_forcefieldUP.get());
    StiffSpringFF* stiffspringforcefield_DOWN = static_cast<StiffSpringFF*>(m_forcefieldDOWN.get());
    //AttachConstraint* attach = static_cast<AttachConstraint*>(m_attach.get());

    int nbrVM1 = m_mord1->getSize();
    int nbrVM2 = m_mord2->getSize();
    for (int i = 0; i < idsModel.size(); i++)
    {
        SReal Mx = m_model->getPX(idsModel[i]);
        SReal My = m_model->getPY(idsModel[i]);
        SReal Mz = m_model->getPZ(idsModel[i]);

        bool attached = false;
        int idModel = -1;
        float minDist = 2;
        // compute bary on mordUP
        for (int j = 0; j < nbrVM1; j++)
        {
            SReal x = m_mord1->getPX(j);
            SReal y = m_mord1->getPY(j);
            SReal z = m_mord1->getPZ(j);
            SReal dist = (Mx - x)*(Mx - x) + (My - y)*(My - y) + (Mz - z)*(Mz - z);
            dist = sqrt(dist);

            if (dist < minDist) {
                minDist = dist;
                idModel = j;
            }
        }

        if (idModel != -1)
        {
            stiffspringforcefield_UP->addSpring(idsModel[i], idModel, 100, 0.0, minDist);
            //attach->addConstraint(idsModel[i], idModel, 1.0);
            m_idgrabed.push_back(idsModel[i]);
        }

        
        //attached = false;
        //// compute bary on mordUP
        //for (int j = 0; j < nbrVM2; j++)
        //{
        //    SReal x = m_mord2->getPX(j);
        //    SReal y = m_mord2->getPY(j);
        //    SReal z = m_mord2->getPZ(j);
        //    SReal dist = (Mx - x)*(Mx - x) + (My - y)*(My - y) + (Mz - z)*(Mz - z);
        //    dist = sqrt(dist);

        //    if (dist < 2) {
        //        stiffspringforcefield_DOWN->addSpring(idsModel[i], j, 100, 0.0, dist);
        //        attached = true;
        //    }
        //}

        //if (attached)
        //    m_idgrabed.push_back(idsModel[i]);

        //// compute bary on mordUP
        //for (int j = 0; j < nbrVM1; j++)
        //{
        //    SReal x = m_mord1->getPX(j);
        //    SReal y = m_mord1->getPY(j);
        //    SReal z = m_mord1->getPZ(j);
        //    SReal dist = (Mx - x)*(Mx - x) + (My - y)*(My - y) + (Mz - z)*(Mz - z);
        //    dist = sqrt(dist);

        //    if (dist < 0.5)
        //        stiffspringforcefield->addSpring(idsModel[i], j, 100, 0.0, dist);
        //}
    }

    sout << m_idgrabed << sendl;

   

    std::cout << "Narrow Phase detection: " << m_idgrabed.size() << std::endl;




    return m_idgrabed;
}

void SleevePinceManager::releaseGrab()
{    
    if (!m_forcefieldUP || !m_forcefieldDOWN)
        return;
    m_idgrabed.clear();

    StiffSpringFF* stiffspringforcefield_UP = static_cast<StiffSpringFF*>(m_forcefieldUP.get());
    stiffspringforcefield_UP->clear();

    StiffSpringFF* stiffspringforcefield_DOWN = static_cast<StiffSpringFF*>(m_forcefieldDOWN.get());
    stiffspringforcefield_DOWN->clear();
}

void SleevePinceManager::createFF()
{
    m_forcefieldUP = sofa::core::objectmodel::New<StiffSpringFF>(dynamic_cast<mechaState*>(m_model), dynamic_cast<mechaState*>(m_mord1));
    StiffSpringFF* stiffspringforcefield_UP = static_cast<StiffSpringFF*>(m_forcefieldUP.get());
    stiffspringforcefield_UP->setName("pince_Forcefield_UP");
    this->getContext()->addObject(stiffspringforcefield_UP);
    stiffspringforcefield_UP->setStiffness(300);
    stiffspringforcefield_UP->setDamping(1);
    stiffspringforcefield_UP->init();

    m_forcefieldDOWN = sofa::core::objectmodel::New<StiffSpringFF>(dynamic_cast<mechaState*>(m_model), dynamic_cast<mechaState*>(m_mord2));
    StiffSpringFF* stiffspringforcefield_DOWN = static_cast<StiffSpringFF*>(m_forcefieldDOWN.get());
    stiffspringforcefield_DOWN->setName("pince_Forcefield_DOWN");
    this->getContext()->addObject(stiffspringforcefield_DOWN);
    stiffspringforcefield_DOWN->init();

    //m_attach = sofa::core::objectmodel::New<AttachConstraint>(dynamic_cast<mechaState*>(m_model), dynamic_cast<mechaState*>(m_mord1));
    //AttachConstraint* attach = static_cast<AttachConstraint*>(m_attach.get());
    //attach->setName("pince_attach");
    //this->getContext()->addObject(attach);
    //attach->init();
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
            if (m_forcefieldUP == NULL || m_forcefieldDOWN == NULL)
                createFF();

            releaseGrab();

            computeBoundingBox();
            grabModel();
            break;
        }
        case 'G':
        case 'g':
        {
            releaseGrab();
            break;
        }
        case 'Y':
        case 'y':
        {
            m_mord1->applyTranslation(0, -0.1, 0);
            m_mord2->applyTranslation(0, 0.1, 0);
            break;
        }
        case 'H':
        case 'h':
        {
            m_mord1->applyTranslation(0, 0.1, 0);
            m_mord2->applyTranslation(0, -0.1, 0);
            break;
        }
        case 'J':
        case 'j':
        {
            m_mord1->applyTranslation(0, 0.1, 0);
            m_mord2->applyTranslation(0, 0.1, 0);
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

    if (m_model == NULL)
        return;

    for (int i = 0; i < m_idgrabed.size(); i++)
    {
        SReal x = m_model->getPX(m_idgrabed[i]);
        SReal y = m_model->getPY(m_idgrabed[i]);
        SReal z = m_model->getPZ(m_idgrabed[i]);
        vparams->drawTool()->drawPoint(sofa::defaulttype::Vec3f(x, y, z), Vec<4, float>(255.0, 0.0, 0.0, 1.0));
    }
        
   // std::cout << "drawLine: " << m_min[0] << " " << m_min[1] << " " << m_min[2] << std::endl;
}



} // namespace misc

} // namespace component

} // namespace sofa
