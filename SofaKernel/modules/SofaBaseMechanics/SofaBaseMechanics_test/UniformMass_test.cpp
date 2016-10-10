/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, development version     *
*                (c) 2006-2016 INRIA, USTL, UJF, CNRS, MGH                    *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU General Public License as published by the Free  *
* Software Foundation; either version 2 of the License, or (at your option)   *
* any later version.                                                          *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for    *
* more details.                                                               *
*                                                                             *
* You should have received a copy of the GNU General Public License along     *
* with this program; if not, write to the Free Software Foundation, Inc., 51  *
* Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.                   *
*******************************************************************************
*                            SOFA :: Applications                             *
*                                                                             *
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#if 0
#include <SofaBaseMechanics/UniformMass.h>
#include <string>
using std::string ;

#include <gtest/gtest.h>
using testing::Types;

#include <sofa/helper/BackTrace.h>
#include <SofaBaseMechanics/MechanicalObject.h>

#include <SofaBaseMechanics/UniformMass.h>
using sofa::component::mass::UniformMass ;

#include <SofaBaseMechanics/initBaseMechanics.h>
using sofa::component::initBaseMechanics ;

#include <SofaSimulationGraph/DAGSimulation.h>
using sofa::simulation::Simulation ;
using sofa::simulation::graph::DAGSimulation ;
using sofa::simulation::Node ;
using sofa::simulation::setSimulation ;
using sofa::core::objectmodel::New ;
using sofa::core::objectmodel::BaseData ;
using sofa::core::ExecParams ;
using sofa::component::container::MechanicalObject ;
using sofa::defaulttype::Vec3dTypes ;

#include <SofaSimulationCommon/SceneLoaderXML.h>
using sofa::simulation::SceneLoaderXML ;

#include <sofa/helper/logging/Message.h>
using sofa::helper::logging::MessageDispatcher ;

#include <sofa/helper/logging/ClangMessageHandler.h>
using sofa::helper::logging::ClangMessageHandler ;

int initMessage(){
    MessageDispatcher::clearHandlers() ;
    MessageDispatcher::addHandler(new ClangMessageHandler()) ;
    return 0;
}
int messageInited = initMessage();

template <class TDataTypes, class TMassTypes>
struct TemplateTypes
{
  typedef TDataTypes DataTypes ;
  typedef TMassTypes MassTypes ;
};

template <typename TTemplateTypes>
struct UniformMassTest : public ::testing::Test
{
    typedef UniformMass<typename TTemplateTypes::DataTypes,
                        typename TTemplateTypes::MassTypes> TheUniformMass ;


    /// Bring parents members in the current lookup context.
    /// more info at: https://gcc.gnu.org/onlinedocs/gcc/Name-lookup.html
    typedef typename TTemplateTypes::DataTypes DataTypes ;
    typedef typename TTemplateTypes::MassTypes MassTypes ;

    Simulation* m_simu {nullptr} ;
    Node::SPtr m_root ;
    Node::SPtr m_node ;
    typename TheUniformMass::SPtr m_mass ;
    typename MechanicalObject<DataTypes>::SPtr m_mecaobject;
    bool todo {true} ;

    virtual void SetUp()
    {
        todo = true ;
        initBaseMechanics();
        setSimulation( m_simu = new DAGSimulation() );
        m_root = m_simu->createNewGraph("root");
    }

    void TearDown()
    {
        if (m_root != NULL){
            m_simu->unload(m_root);
        }
    }

    /// It is important to freeze what are the available Data field
    /// of a component and rise warning/errors when some one removed.
    ///
    void attributesTests(){
        m_node = m_root->createChild("node") ;
        m_mass = New< TheUniformMass >() ;
        m_node->addObject(m_mass) ;

        EXPECT_TRUE( m_mass->findData("mass") != nullptr ) ;
        EXPECT_TRUE( m_mass->findData("totalmass") != nullptr ) ;
        EXPECT_TRUE( m_mass->findData("filename") != nullptr ) ;
        EXPECT_TRUE( m_mass->findData("localRange") != nullptr ) ;

        EXPECT_TRUE( m_mass->findData("showGravityCenter") != nullptr ) ;
        EXPECT_TRUE( m_mass->findData("showAxisSizeFactor") != nullptr ) ;
        EXPECT_TRUE( m_mass->findData("showInitialCenterOfGravity") != nullptr ) ;

        EXPECT_TRUE( m_mass->findData("indices") != nullptr ) ;
        EXPECT_TRUE( m_mass->findData("handleTopoChange") != nullptr ) ;
        EXPECT_TRUE( m_mass->findData("preserveTotalMass") != nullptr ) ;

        EXPECT_TRUE( m_mass->findData("compute_mapping_inertia") != nullptr ) ;
        EXPECT_TRUE( m_mass->findData("totalMass") != nullptr ) ;
        return ;
    }

    /// totalMass, mass and localRange..
    /// totalMass & mass are exclusive.
    /// si mass and total mass set c'est total mass le plus fort.
    void checkCreationOfUniformMassRigid(){
        string scene =
                "<?xml version='1.0'?>"
                "<Node 	name='Root' gravity='0 0 0' time='0' animate='0'   >       "
                "   <MechanicalObject position='0 0 0 4 5 6' template='Rigid3d'/>  "
                "   <UniformMass template='Rigid3d' name='m_massd'/>          "
                "</Node>                                                           " ;

        Node::SPtr root = SceneLoaderXML::loadFromMemory ("loadRigidWithNoParam",
                                                          scene.c_str(),
                                                          scene.size()) ;

        root->init(ExecParams::defaultInstance()) ;

        sofa::core::objectmodel::BaseObject* massd = root->getObject("m_massd") ;
        EXPECT_TRUE( massd != nullptr ) ;

        massd->reinit() ;
        if( massd!=nullptr ){
            std::cout << "MASS 1: " << massd->getTemplateName() << std::endl ;

        }
    }

    /// totalMass, mass and localRange..
    /// totalMass & mass are exclusive.
    /// si mass and total mass set c'est total mass le plus fort.
    void checkCreationOfUniformMassVec3d(){
        string scene =
                "<?xml version='1.0'?>"
                "<Node 	name='Root' gravity='0 0 0' time='0' animate='0'   >       "
                "   <MechanicalObject position='0 0 0 4 5 6' template='Vec3d'/>  "
                "   <UniformMass template='Vec3d' name='m_massd'/>          "
                "</Node>                                                           " ;

        Node::SPtr root = SceneLoaderXML::loadFromMemory ("loadRigidWithNoParam",
                                                          scene.c_str(),
                                                          scene.size()) ;

        root->init(ExecParams::defaultInstance()) ;

        sofa::core::objectmodel::BaseObject* massd = root->getObject("m_massd") ;
        EXPECT_TRUE( massd != nullptr ) ;

        massd->reinit() ;
        if( massd!=nullptr ){
            std::cout << "MASS 1: " << massd->getTemplateName() << std::endl ;

        }
    }


    /// totalMass, mass and localRange..
    /// totalMass & mass are exclusive.
    /// si mass and total mass set c'est total mass le plus fort.
    void checkDefaultValuesForAttributes(){
        string scene =
                "<?xml version='1.0'?>"
                "<Node 	name='Root' gravity='0 0 0' time='0' animate='0'   > "
                "   <MechanicalObject position='0 0 0 4 5 6'/>               "
                "   <UniformMass name='m_mass'/>                             "
                "</Node>                                                     " ;

        Node::SPtr root = SceneLoaderXML::loadFromMemory ("loadWithNoParam",
                                                          scene.c_str(),
                                                          scene.size()) ;

        root->init(ExecParams::defaultInstance()) ;

        TheUniformMass* mass = root->getTreeObject<TheUniformMass>() ;
        EXPECT_TRUE( mass != nullptr ) ;

        if(mass!=nullptr){
            EXPECT_EQ( mass->getMass(), 1.0 ) ;
            EXPECT_EQ( mass->getTotalMass(), 2.0 ) ;
        }
    }

    /// totalMass, mass and localRange..
    void checkMassTotalFromMass(){
        string scene =
                "<?xml version='1.0'?>"
                "<Node 	name='Root' gravity='0 0 0' time='0' animate='0'   > "
                "   <MechanicalObject position='0 0 0 4 5 6'/>               "
                "   <UniformMass name='m_mass' mass='4.0' />                 "
                "</Node>                                                     " ;

        Node::SPtr root = SceneLoaderXML::loadFromMemory ("loadWithNoParam",
                                                          scene.c_str(),
                                                          (int)scene.size()) ;

        root->init(ExecParams::defaultInstance()) ;

        TheUniformMass* mass = root->getTreeObject<TheUniformMass>() ;
        EXPECT_TRUE( mass != nullptr ) ;

        if(mass!=nullptr){
            EXPECT_EQ( mass->getMass(), 4.0 ) ;
            EXPECT_EQ( mass->getTotalMass(), 8.0 ) ;
        }
    }

    /// totalMass, mass and localRange..
    void checkMassFromMassTotal(){
        string scene =
                "<?xml version='1.0'?>"
                "<Node 	name='Root' gravity='0 0 0' time='0' animate='0'   > "
                "   <MechanicalObject position='0 0 0 4 5 6'/>               "
                "   <UniformMass name='m_mass' totalmass='4.0' />            "
                "</Node>                                                     " ;

        Node::SPtr root = SceneLoaderXML::loadFromMemory ("loadWithNoParam",
                                                          scene.c_str(),
                                                          (int)scene.size()) ;

        root->init(ExecParams::defaultInstance()) ;

        TheUniformMass* mass = root->getTreeObject<TheUniformMass>() ;
        EXPECT_TRUE( mass != nullptr ) ;

        if(mass!=nullptr){
            EXPECT_EQ( mass->getMass(), 2.0 ) ;
            EXPECT_EQ( mass->getTotalMass(), 4.0 ) ;
        }
    }

    /// totalMass, mass and localRange..
    void checkMassAndMassTotal(){
        string scene =
                "<?xml version='1.0'?>"
                "<Node 	name='Root' gravity='0 0 0' time='0' animate='0'   > "
                "   <MechanicalObject position='0 0 0 4 5 6'/>               "
                "   <UniformMass name='m_mass' totalmass='91.0' mass=2.0/>   "
                "</Node>                                                     " ;

        Node::SPtr root = SceneLoaderXML::loadFromMemory ("loadWithNoParam",
                                                          scene.c_str(),
                                                          (int)scene.size()) ;

        root->init(ExecParams::defaultInstance()) ;

        TheUniformMass* mass = root->getTreeObject<TheUniformMass>() ;
        EXPECT_TRUE( mass != nullptr ) ;

        if(mass!=nullptr){
            EXPECT_EQ( mass->getMass(), 45.5 ) ;
            EXPECT_EQ( mass->getTotalMass(), 91.0 ) ;
        }
    }

    /// if masses are negative we refuse them and use the default values.
    void checkNegativeMassNotAllowed(){
        string scene =
                "<?xml version='1.0'?>"
                "<Node 	name='Root' gravity='0 0 0' time='0' animate='0'   > "
                "   <MechanicalObject position='0 0 0 4 5 6'/>               "
                "   <UniformMass name='m_mass' totalmass='-1.0' mass=-3.0/>   "
                "</Node>                                                     " ;

        Node::SPtr root = SceneLoaderXML::loadFromMemory ("loadWithNoParam",
                                                          scene.c_str(),
                                                          scene.size()) ;

        root->init(ExecParams::defaultInstance()) ;

        TheUniformMass* mass = root->getTreeObject<TheUniformMass>() ;
        EXPECT_TRUE( mass != nullptr ) ;

        std::cout << "HELLOW WORLD" << mass->getClassName() << std::endl ;
    }

    void loadFromAFileForNonRigid(){
        string scene =
                "<?xml version='1.0'?>"
                "<Node 	name='Root' gravity='0 0 0' time='0' animate='0'   > "
                "   <MechanicalObject position='0 0 0'/>                     "
                "   <UniformMass filename='valid_uniformmatrix.txt'/>        "
                "</Node>                                                     " ;
        Node::SPtr root = SceneLoaderXML::loadFromMemory ("loadFromAValidFile",
                                                          scene.c_str(), (int)scene.size()) ;
        root->init(ExecParams::defaultInstance()) ;
    }

    void loadFromAnInvalidFile(){
        string scene =
                "<?xml version='1.0'?>"
                "<Node 	name='Root' gravity='0 0 0' time='0' animate='0'   > "
                "   <MechanicalObject position='0 0 0'/>                     "
                "   <UniformMass filename='valid_uniformmatrix.txt'/>        "
                "</Node>                                                     " ;
        Node::SPtr root = SceneLoaderXML::loadFromMemory ("loadFromAValidFile",
                                                          scene.c_str(), (int)scene.size()) ;
        root->init(ExecParams::defaultInstance()) ;
    }

    void loadFromAnInvalidPathname(){
        // TODO
        EXPECT_TRUE(todo == false) ;
    }

    void reinitTest(){
        // TODO
        EXPECT_TRUE(todo == false) ;
    }

};

typedef Types< TemplateTypes<Vec3dTypes, double>


             > DataTypes;

TYPED_TEST_CASE(UniformMassTest, DataTypes);


TYPED_TEST(UniformMassTest, attributesTests) {
    ASSERT_NO_THROW(this->attributesTests()) ;
}

TYPED_TEST(UniformMassTest, checkCreationOfUniformMassRigid) {
    ASSERT_NO_THROW(this->checkCreationOfUniformMassRigid()) ;
}

TYPED_TEST(UniformMassTest, checkMassTotalFromMass)
{
    ASSERT_NO_THROW(this->checkMassTotalFromMass()) ;
}

TYPED_TEST(UniformMassTest, checkMassFromMassTotal)
{
    ASSERT_NO_THROW(this->checkMassFromMassTotal()) ;
}

TYPED_TEST(UniformMassTest, checkMassAndMassTotal)
{
    ASSERT_NO_THROW(this->checkMassAndMassTotal()) ;
}

TYPED_TEST(UniformMassTest, checkNegativeMassNotAllowed)
{
    ASSERT_NO_THROW(this->checkNegativeMassNotAllowed()) ;
}

TYPED_TEST(UniformMassTest, checkDefaultValuesForAttributes) {
    ASSERT_NO_THROW(this->checkDefaultValuesForAttributes()) ;
}

TYPED_TEST(UniformMassTest, loadFromAFileForNonRigid) {
    ASSERT_NO_THROW(this->loadFromAFileForNonRigid()) ;
}

TYPED_TEST(UniformMassTest, loadFromAnInvalidFile) {
    //ASSERT_NO_THROW(this->loadFromAnInvalidFile()) ;
}

TYPED_TEST(UniformMassTest, loadFromAnInvalidPathname) {
    ASSERT_NO_THROW(this->loadFromAnInvalidPathname()) ;
}

TYPED_TEST(UniformMassTest, reinitTest) {
    //ASSERT_NO_THROW(this->reinitTest()) ;
}

#endif //

