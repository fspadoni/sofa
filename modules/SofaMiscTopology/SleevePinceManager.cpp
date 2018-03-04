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

#include <SofaBaseTopology/TetrahedronSetTopologyContainer.h>
#include <SofaBaseTopology/TetrahedronSetTopologyModifier.h>

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
using namespace sofa::core::topology;

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

void SleevePinceManager::computeVertexIdsInBroadPhase()
{
    m_idBroadPhase.clear();

    if (m_model == NULL)
        return;

    for (int i = 0; i < m_model->getSize(); i++)
    {
        SReal x = m_model->getPX(i);
        SReal y = m_model->getPY(i);
        SReal z = m_model->getPZ(i);
        if (x > m_min[0] && x < m_max[0]
            && y > m_min[1] && y < m_max[1]
            && z > m_min[2] && z < m_max[2])
        {
            m_idBroadPhase.push_back(i);
        }
    }
}

const sofa::helper::vector< int >& SleevePinceManager::grabModel()
{
    if (m_idBroadPhase.size() == 0)
        return m_idBroadPhase;


    // Second distance to spheres

    StiffSpringFF* stiffspringforcefield_UP = static_cast<StiffSpringFF*>(m_forcefieldUP.get());
    StiffSpringFF* stiffspringforcefield_DOWN = static_cast<StiffSpringFF*>(m_forcefieldDOWN.get());
    //AttachConstraint* attach = static_cast<AttachConstraint*>(m_attach.get());

    int nbrVM1 = m_mord1->getSize();
    int nbrVM2 = m_mord2->getSize();
    for (int i = 0; i < m_idBroadPhase.size(); i++)
    {
        SReal Mx = m_model->getPX(m_idBroadPhase[i]);
        SReal My = m_model->getPY(m_idBroadPhase[i]);
        SReal Mz = m_model->getPZ(m_idBroadPhase[i]);

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
            stiffspringforcefield_UP->addSpring(m_idBroadPhase[i], idModel, 100, 0.0, minDist);
            //attach->addConstraint(idsModel[i], idModel, 1.0);
            m_idgrabed.push_back(m_idBroadPhase[i]);
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
    m_idBroadPhase.clear();

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

void SleevePinceManager::computePlierAxis()
{
    zero = sofa::defaulttype::Vec3f(0, 0, 0);
    xAxis = sofa::defaulttype::Vec3f(1, 0, 0);
    yAxis = sofa::defaulttype::Vec3f(0, 1, 0);
    zAxis = sofa::defaulttype::Vec3f(0, 0, 1);

    if (m_mord1 == NULL)
        return;

    zero = sofa::defaulttype::Vec3f(m_mord1->getPX(0), m_mord1->getPY(0), m_mord1->getPZ(0));
    xAxis = sofa::defaulttype::Vec3f(m_mord1->getPX(1), m_mord1->getPY(1), m_mord1->getPZ(1));
    yAxis = sofa::defaulttype::Vec3f(m_mord1->getPX(20), m_mord1->getPY(20), m_mord1->getPZ(20));
    zAxis = sofa::defaulttype::Vec3f(m_mord1->getPX(100), m_mord1->getPY(100), m_mord1->getPZ(100));

    sofa::defaulttype::Vec3f xDir = (xAxis - zero); xDir.normalize();
    sofa::defaulttype::Vec3f yDir = (yAxis - zero); yDir.normalize();
    sofa::defaulttype::Vec3f zDir = (zAxis - zero); zDir.normalize();

    matP = sofa::defaulttype::Mat3x3f(xDir, yDir, zDir);

    //sofa::defaulttype::Vec3f test1 = sofa::defaulttype::Vec3f(m_mord1->getPX(3), m_mord1->getPY(3), m_mord1->getPZ(3));
    //sofa::defaulttype::Vec3f test2 = sofa::defaulttype::Vec3f(m_mord1->getPX(40), m_mord1->getPY(40), m_mord1->getPZ(40));
    //sofa::defaulttype::Vec3f test3 = sofa::defaulttype::Vec3f(m_mord1->getPX(45), m_mord1->getPY(45), m_mord1->getPZ(45));
    //std::cout << "test1 : " << test1 << " -> " << matP*(test1 - zero) << std::endl;
    //std::cout << "test2 : " << test2 << " -> " << matP*(test2 - zero) << std::endl;
    //std::cout << "test3 : " << test3 << " -> " << matP*(test3 - zero) << std::endl;



    
}

void SleevePinceManager::cutFromTetra()
{
    if (m_idBroadPhase.empty())
        return;

    // Classify right/left points of the plier
    sofa::helper::vector<int> idsLeft;
    sofa::helper::vector<int> idsRight;
    for (int i = 0; i < m_idBroadPhase.size(); i++)
    {
        sofa::defaulttype::Vec3f vert = sofa::defaulttype::Vec3f(m_model->getPX(m_idBroadPhase[i]), m_model->getPY(m_idBroadPhase[i]), m_model->getPZ(m_idBroadPhase[i]));
        vert = matP*(vert - zero);

        if (vert[0] < 0.0 || vert[0] > 8.0)
            continue;

        if (vert[2] >= -2.0 && vert[2] < 1.0)
            idsLeft.push_back(m_idBroadPhase[i]);
        else if (vert[2] >= 1.0 && vert[2] < 3.0)
            idsRight.push_back(m_idBroadPhase[i]);
    }

    std::cout << "idsLeft: " << idsLeft.size() << std::endl;
    std::cout << "idsRight: " << idsRight.size() << std::endl;

    // Detect all tetra on the cut path
    sofa::component::topology::TetrahedronSetTopologyContainer* tetraCon;
    m_model->getContext()->get(tetraCon);
    if (tetraCon == NULL) {
        std::cout << "Error: NO tetraCon" << std::endl;
        return;
    }

    // First get all tetra that are on the first side
    sofa::helper::vector<unsigned int> tetraIds;
    for (int i = 0; i < idsLeft.size(); ++i)
    {
        const BaseMeshTopology::TetrahedraAroundVertex& tetraAV = tetraCon->getTetrahedraAroundVertex(idsLeft[i]);
        for (int j = 0; j < tetraAV.size(); ++j)
        {
            int tetraId = tetraAV[j];
            bool found = false;
            for (int k = 0; k<tetraIds.size(); ++k)
                if (tetraIds[k] == tetraId)
                {
                    found = true;
                    break;
                }

            if (!found)
                tetraIds.push_back(tetraId);
        }
    }

    std::cout << "tetraIds: " << tetraIds.size() << std::endl;


    // Then test for each tetra if one of the vertex is on the other side. If yes put on but path
    tetraIdsOnCut.clear();
    std::set< unsigned int > items;
    for (int i = 0; i < tetraIds.size(); ++i)
    {
        const BaseMeshTopology::Tetra& tetra = tetraCon->getTetra(tetraIds[i]);
        for (unsigned int j = 0; j < 4; ++j)
        {
            int idV = tetra[j];
            bool found = false;
            for (unsigned int k = 0; k < idsRight.size(); ++k)
            {
                if (idsRight[k] == idV)
                {
                    found = true;
                    break;
                }
            }

            if (found) {
                tetraIdsOnCut.push_back(tetraIds[i]);
                items.insert(tetraIds[i]);
                continue;
            }
        }
    }

    std::cout << "tetraIdsOnCut: " << tetraIdsOnCut.size() << std::endl;
    sofa::component::topology::TetrahedronSetTopologyModifier* tetraModif;
    m_model->getContext()->get(tetraModif);

    if (tetraModif == NULL) {
        std::cout << "Error: NO tetraModif" << std::endl;
        return;
    }


    sofa::helper::vector<unsigned int> vitems;
    vitems.reserve(items.size());
    vitems.insert(vitems.end(), items.rbegin(), items.rend());

    //vitems.resize(30);
    tetraModif->removeTetrahedra(vitems);
    tetraModif->notifyEndingEvent();
    tetraModif->propagateTopologicalChanges();
}


void SleevePinceManager::cutFromTriangles()
{
    // Classify right/left points of the plier
    sofa::helper::vector<int> idsLeft;
    sofa::helper::vector<int> idsRight;
    for (int i = 0; i < m_idgrabed.size(); i++)
    {
        sofa::defaulttype::Vec3f vert = sofa::defaulttype::Vec3f(m_model->getPX(m_idgrabed[i]), m_model->getPY(m_idgrabed[i]), m_model->getPZ(m_idgrabed[i]));
        vert = matP*(vert - zero);

        if (vert[0] < 0.0 || vert[0] > 8.0)
            continue;

        if (vert[2] >= -2.0 && vert[2] < 1.0)
            idsLeft.push_back(m_idgrabed[i]);
        else if (vert[2] >= 1.0 && vert[2] < 3.0)
            idsRight.push_back(m_idgrabed[i]);
    }

    std::cout << "idsLeft: " << idsLeft.size() << std::endl;
    std::cout << "idsRight: " << idsRight.size() << std::endl;

    // Detect all tetra on the cut path
    std::vector<sofa::component::topology::TriangleSetTopologyContainer*> triCons;
    m_model->getContext()->get<sofa::component::topology::TriangleSetTopologyContainer>(&triCons, sofa::core::objectmodel::BaseContext::SearchDown);

    if (triCons.size() < 2) {
        std::cout << "Error: NO triCons" << std::endl;
        return;
    }

    const sofa::helper::vector<BaseMeshTopology::Triangle> & allTri = triCons[1]->getTriangleArray();
    for (int i = 0; i < allTri.size(); ++i)
    {
        const BaseMeshTopology::Triangle& tri = allTri[i];
        bool foundLeft = false;
        bool foundRight = false;
        for (int j = 0; j < 3; ++j)
        {
            int idV = tri[j];
            for (int k = 0; k < idsLeft.size(); ++k)
            {
                if (idsLeft[k] == idV)
                {
                    foundLeft = true;
                    break;
                }
            }

            if (foundLeft)
                break;
        }

        if (!foundLeft)
            continue;

        std::cout << "found: " << i << std::endl;
        for (int j = 0; j < 3; ++j)
        {
            int idV = tri[j];
            for (int k = 0; k < idsRight.size(); ++k)
            {
                if (idsRight[k] == idV)
                {
                    foundRight = true;
                    break;
                }
            }

            if (foundRight) {
                triIdsOnCut.push_back(i);
                break;
            }
        }
    }

    std::cout << "triIdsOnCut: " << triIdsOnCut.size() << std::endl;
    std::vector<sofa::component::topology::TriangleSetTopologyModifier*> triModifs;
    m_model->getContext()->get<sofa::component::topology::TriangleSetTopologyModifier>(&triModifs, sofa::core::objectmodel::BaseContext::SearchDown);

    if (triModifs.size() < 2 ) {
        std::cout << "Error: NO triModif" << std::endl;
        return;
    }
    std::cout << "FOUND: " << triModifs.size() << std::endl;
    //tetraIdsOnCut.resize(30);
    triModifs[1]->removeItems(triIdsOnCut);
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
            computeVertexIdsInBroadPhase();
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
        case 'F':
        case 'f':
        {
            computeBoundingBox();
            computeVertexIdsInBroadPhase();
            computePlierAxis();
            //cutFromTriangles();
            cutFromTetra();
            break;
        }
        }
    }
}

void SleevePinceManager::draw(const core::visual::VisualParams* vparams)
{
    if (!vparams->displayFlags().getShowBehaviorModels())
        return;

    sofa::defaulttype::Vec4f color = sofa::defaulttype::Vec4f(0.2f, 1.0f, 1.0f, 1.0f);
    vparams->drawTool()->drawLine(m_min, m_max, Vec<4, float>(1.0, 0.0, 1.0, 1.0));
    
    glColor3f(1.0, 0.0, 0.0);
    vparams->drawTool()->drawLine(zero, xAxis, color);
    glColor3f(0.0, 1.0, 0.0);
    vparams->drawTool()->drawLine(zero, yAxis, color);
    glColor3f(0.0, 0.0, 1.0);
    vparams->drawTool()->drawLine(zero, zAxis, color);
    glColor3f(1.0, 1.0, 1.0);

    if (m_model == NULL)
        return;

    for (int i = 0; i < m_idgrabed.size(); i++)
    {
        SReal x = m_model->getPX(m_idgrabed[i]);
        SReal y = m_model->getPY(m_idgrabed[i]);
        SReal z = m_model->getPZ(m_idgrabed[i]);
        vparams->drawTool()->drawPoint(sofa::defaulttype::Vec3f(x, y, z), Vec<4, float>(255.0, 0.0, 0.0, 1.0));
    }


    sofa::component::topology::TetrahedronSetTopologyContainer* tetraCon;
    m_model->getContext()->get(tetraCon);
    if (tetraCon == NULL) {
        std::cout << "Error: NO tetraCon" << std::endl;
        return;
    }

    for (int i = 0; i < tetraIdsOnCut.size(); i++)
    {
        const BaseMeshTopology::Tetra& tetra = tetraCon->getTetra(tetraIdsOnCut[i]);
        
        sofa::defaulttype::Vec3f p0 = sofa::defaulttype::Vec3f(m_model->getPX(tetra[0]), m_model->getPY(tetra[0]), m_model->getPZ(tetra[0]));
        sofa::defaulttype::Vec3f p1 = sofa::defaulttype::Vec3f(m_model->getPX(tetra[1]), m_model->getPY(tetra[1]), m_model->getPZ(tetra[1]));
        sofa::defaulttype::Vec3f p2 = sofa::defaulttype::Vec3f(m_model->getPX(tetra[2]), m_model->getPY(tetra[2]), m_model->getPZ(tetra[2]));
        sofa::defaulttype::Vec3f p3 = sofa::defaulttype::Vec3f(m_model->getPX(tetra[3]), m_model->getPY(tetra[3]), m_model->getPZ(tetra[3]));

        vparams->drawTool()->drawTetrahedron(p0, p1, p2, p3, color);
    }

        
   // std::cout << "drawLine: " << m_min[0] << " " << m_min[1] << " " << m_min[2] << std::endl;
}



} // namespace misc

} // namespace component

} // namespace sofa
