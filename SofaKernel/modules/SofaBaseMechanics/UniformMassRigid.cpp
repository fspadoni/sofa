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
#include <iostream>
#include <string.h>

#include<sofa/defaulttype/BaseMatrix.h>
using sofa::defaulttype::BaseMatrix ;

#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/DataTypeInfo.h>


#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/topology/Topology.h>
#include <sofa/core/objectmodel/Context.h>
#include <sofa/helper/accessor.h>
#include <SofaBaseMechanics/AddMToMatrixFunctor.h>
#include <sofa/simulation/Simulation.h>

#include <SofaBaseMechanics/UniformMass.inl>
using sofa::component::mass::UniformMass ;

#include <SofaBaseMechanics/UniformMassRigid.h>

#include <sofa/defaulttype/VecTypes.h>
using sofa::component::mass::Vec3d ;

#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/core/ObjectFactory.h>

using namespace sofa::defaulttype;


#include <sofa/helper/system/FileRepository.h>
using sofa::helper::system::DataRepository ;

#include <sofa/helper/system/Locale.h>

#include <sstream>
using std::string ;
using std::ostringstream ;





namespace sofa
{

namespace component
{

namespace mass
{

static void skipToEOL(FILE* f)
{
    int	ch;
    while ((ch = fgetc(f)) != EOF && ch != '\n')
        ;
}

Mat3x3d MatrixFromEulerXYZ(double thetaX, double thetaY, double thetaZ)
{
    Quatd q=Quatd::fromEuler(thetaX, thetaY, thetaZ) ;
    Mat3x3d m;
    q.toMatrix(m);
    return m;
}

template <class DataTypes, class MassType>
UniformMassRigid<DataTypes, MassType>::UniformMassRigid() : UniformMass<DataTypes, MassType>(){

}

template <class DataTypes, class MassType>
UniformMassRigid<DataTypes, MassType>::~UniformMassRigid(){}

template <class RigidType, class RigidMass> SOFA_BASE_MECHANICS_API
void UniformMassRigid<RigidType, RigidMass>::reinit()
{
    if ( d_filenameMass.isSet() && d_filenameMass.getValue() != "unused" ){
        loadRigidMass(d_filenameMass.getFullPath()) ;
    }

    Inherit1::reinit() ;

    d_mass.beginEdit()->recalc();
    d_mass.endEdit();
}

template<class RigidMass>
void dirtyHackToGetInertiaMatrix(RigidMass& m, Mat<3,3,double>& tmp){
    m.inertiaMatrix = tmp ;
}

template<>
void dirtyHackToGetInertiaMatrix(Rigid2fMass& m, Mat<3,3,double>& tmp){
    return m = tmp[0][0];
}

template<>
void dirtyHackToGetInertiaMatrix(Rigid2dMass& m, Mat<3,3,double>& tmp){
    return m = tmp[0][0];
}

template <class RigidType, class RigidMass>
SOFA_BASE_MECHANICS_API
void UniformMassRigid<RigidType, RigidMass>::loadRigidMass(string filename)
{
    if(RigidType::spatial_dimensions != 3){
        if (!filename.empty())
        {
            msg_warning(this) << "Trying to load the mass description from file ["+filename+"]\n"
                                 "This function only works for Rigid3 objects while this one is "<< templateName(this) << "\n"
                                 "To remove this warning message you need to let the filename='' attributes empty.\n" ;
            return ;
        }
    }

    if (!filename.empty())
    {
        RigidMass m = getMass();
        if (!DataRepository.findFile(filename))
            serr << "ERROR: cannot find file '" << filename << "'." << sendl;
        else
        {
            char	cmd[64];
            FILE*	file;
            if ((file = fopen(filename.c_str(), "r")) == NULL)
            {
                serr << "ERROR: cannot read file '" << filename << "'." << sendl;
            }
            else
            {
                {
                    skipToEOL(file);
                    ostringstream cmdScanFormat;
                    cmdScanFormat << "%" << (sizeof(cmd) - 1) << "s";
                    while (fscanf(file, cmdScanFormat.str().c_str(), cmd) != EOF)
                    {
                        if (!strcmp(cmd,"inrt"))
                        {
                            Mat<3,3, double> tmp;
                            for (int i = 0; i < 3; i++){
                                for (int j = 0; j < 3; j++){

                                    if( fscanf(file, "%lf", &(tmp[i][j])) < 1 )
                                        serr << SOFA_CLASS_METHOD << "error reading file '" << filename << "'." << sendl;
                                }
                            }
                            dirtyHackToGetInertiaMatrix(m, tmp) ;
                        }
                        else if (!strcmp(cmd,"cntr") || !strcmp(cmd,"center") )
                        {
                            Vec3d center;
                            for (int i = 0; i < 3; ++i)
                            {
                                double tmp;
                                if( fscanf(file, "%lf", &(tmp)) < 1 )
                                    serr << SOFA_CLASS_METHOD << "error reading file '" << filename << "'." << sendl;
                                center[i] = tmp ;
                            }
                        }
                        else if (!strcmp(cmd,"mass"))
                        {
                            double mass=0.0;
                            if( fscanf(file, "%lf", &mass) > 0 )
                            {
                                if (!this->d_mass.isSet())
                                    m.mass = mass;
                            }
                            else
                                serr << SOFA_CLASS_METHOD << "error reading file '" << filename << "'." << sendl;
                        }
                        else if (!strcmp(cmd,"volm"))
                        {
                            double tmp=0.0;
                            if( fscanf(file, "%lf", &(tmp)) < 1 )
                                serr << SOFA_CLASS_METHOD << "error reading file '" << filename << "'." << sendl;
                            m.volume=tmp;
                        }
                        else if (!strcmp(cmd,"frme"))
                        {
                            Quatd orient;
                            for (int i = 0; i < 4; ++i)
                            {
                                if( fscanf(file, "%lf", &(orient[i])) < 1 )
                                    serr << SOFA_CLASS_METHOD << "error reading file '" << filename << "'." << sendl;
                            }
                            orient.normalize();
                        }
                        else if (!strcmp(cmd,"grav"))
                        {
                            Vec3d gravity;
                            if( fscanf(file, "%lf %lf %lf\n", &(gravity.x()), &(gravity.y()), &(gravity.z())) < 3 )
                                serr << SOFA_CLASS_METHOD << "error reading file '" << filename << "'." << sendl;
                        }
                        else if (!strcmp(cmd,"visc"))
                        {
                            double viscosity = 0;
                            if( fscanf(file, "%lf", &viscosity) < 1 )
                                serr << SOFA_CLASS_METHOD << "error reading file '" << filename << "'." << sendl;

                        }
                        else if (!strcmp(cmd,"stck"))
                        {
                            double tmp;
                            if( fscanf(file, "%lf", &tmp) < 1 ) //&(MSparams.default_stick));
                                serr << SOFA_CLASS_METHOD << "error reading file '" << filename << "'." << sendl;
                        }
                        else if (!strcmp(cmd,"step"))
                        {
                            double tmp;
                            if( fscanf(file, "%lf", &tmp) < 1 ) //&(MSparams.default_dt));
                                serr << SOFA_CLASS_METHOD << "error reading file '" << filename << "'." << sendl;
                        }
                        else if (!strcmp(cmd,"prec"))
                        {
                            double tmp;
                            if( fscanf(file, "%lf", &tmp) < 1 ) //&(MSparams.default_prec));
                            {
                                serr << SOFA_CLASS_METHOD << "error reading file '" << filename << "'." << sendl;
                            }
                        }
                        else if (cmd[0] == '#')	// it's a comment
                        {
                            skipToEOL(file);
                        }
                        else		// it's an unknown keyword
                        {
                            printf("%s: Unknown RigidMass keyword: %s\n", filename.c_str(), cmd);
                            skipToEOL(file);
                        }
                    }
                }
                fclose(file);
            }
        }
        setMass(m);
    }
    else if (d_totalMass.getValue()>0 && mstate!=NULL) d_mass.setValue((Real)d_totalMass.getValue() / mstate->getSize());
}

template <class RigidType, class RigidMass> SOFA_BASE_MECHANICS_API
SReal UniformMassRigid<RigidType,RigidMass>::getPotentialEnergy( const MechanicalParams*,
                                                                 const DataVecCoord& d_x ) const
{
    SReal e = 0;
    ReadAccessor< DataVecCoord > x = d_x;
    ReadAccessor<Data<vector<int> > > indices = d_indices;

    typename RigidType::CPos g ( getContext()->getGravity() );
    for (unsigned int i=0; i<indices.size(); i++)
        e -= g*d_mass.getValue().mass*x[indices[i]].getCenter();

    return e;
}




template <class RigidType, class RigidMass> SOFA_BASE_MECHANICS_API
Vector6 UniformMassRigid<RigidType,RigidMass>::getMomentum ( const MechanicalParams*,
                                                             const DataVecCoord& d_x,
                                                             const DataVecDeriv& d_v ) const
{
    if(RigidType::spatial_dimensions != 3){
        if(RigidType::spatial_dimensions != 3){
                msg_warning(this) << "Invalid Calling to the getMomentum() function. \n"
                                     "The current implementation only works for Rigid3(df) templates.\n"
                                     "The current template is " << templateName(this) << "\n" ;
                return defaulttype::Vec6d();
        }
    }
    ReadAccessor<DataVecDeriv> v = d_v;
    ReadAccessor<DataVecCoord> x = d_x;
    ReadAccessor<Data<vector<int> > > indices = d_indices;

    Real m = d_mass.getValue().mass;
    const typename RigidMass::TheMat& I = d_mass.getValue().inertiaMassMatrix;

    defaulttype::Vec6d momentum;
    for ( unsigned int i=0 ; i<indices.size() ; i++ )
    {
        typename RigidMass::VecPos linearMomentum = m*v[indices[i]].getLinear();
        for( int j=0 ; j<RigidMass::VecPos::spatial_dimensions ; ++j )
            momentum[j] += linearMomentum[j];

        typename RigidMass::VecOri angularMomentum = typename RigidMass::VecOri(cross( x[indices[i]].getCenter(), linearMomentum )) + ( I * typename RigidMass::VecOri(v[indices[i]].getAngular()) );
        for( int j=0 ; j<RigidMass::VecOri::spatial_dimensions ; ++j )
            momentum[3+j] += angularMomentum[j];
    }

    return momentum;
}

template <class RigidType, class RigidMass> SOFA_BASE_MECHANICS_API
void UniformMassRigid<RigidType, RigidMass>::draw(const VisualParams* vparams)
{
    drawImpl<RigidType>(vparams, 1) ;
}

template <class RigidType, class RigidMass>
template <class P>
SOFA_BASE_MECHANICS_API
void UniformMassRigid<RigidType, RigidMass>::drawImpl(const VisualParams* vparams,
                                                      typename std::enable_if<P::spatial_dimensions == 3, int>::type)
{
    if (!vparams->displayFlags().getShowBehaviorModels())
        return;

    const VecCoord& x =mstate->read(core::ConstVecCoordId::position())->getValue();
    ReadAccessor<Data<vector<int> > > indices = d_indices;
    RigidTypes::Vec3 gravityCenter;
    defaulttype::Vec3d len;

    // The moment of inertia of a box is:
    //   m->_I(0,0) = M/REAL(12.0) * (ly*ly + lz*lz);
    //   m->_I(1,1) = M/REAL(12.0) * (lx*lx + lz*lz);
    //   m->_I(2,2) = M/REAL(12.0) * (lx*lx + ly*ly);
    // So to get lx,ly,lz back we need to do
    //   lx = sqrt(12/M * (m->_I(1,1)+m->_I(2,2)-m->_I(0,0)))
    // Note that RigidMass inertiaMatrix is already divided by M
    double m00 = d_mass.getValue().inertiaMatrix[0][0];
    double m11 = d_mass.getValue().inertiaMatrix[1][1];
    double m22 = d_mass.getValue().inertiaMatrix[2][2];
    len[0] = sqrt(m11+m22-m00);
    len[1] = sqrt(m00+m22-m11);
    len[2] = sqrt(m00+m11-m22);

    for (unsigned int i=0; i<indices.size(); i++)
    {
        if (getContext()->isSleeping())
            vparams->drawTool()->drawFrame(x[indices[i]].getCenter(), x[indices[i]].getOrientation(), len*d_showAxisSize.getValue(), Vec4f(0.5,0.5,0.5,1) );
        else
            vparams->drawTool()->drawFrame(x[indices[i]].getCenter(), x[indices[i]].getOrientation(), len*d_showAxisSize.getValue() );
        gravityCenter += (x[indices[i]].getCenter());
    }

    if (d_showInitialCenterOfGravity.getValue())
    {
        const VecCoord& x0 = mstate->read(core::ConstVecCoordId::restPosition())->getValue();

        for (unsigned int i=0; i<indices.size(); i++)
            vparams->drawTool()->drawFrame(x0[indices[i]].getCenter(), x0[indices[i]].getOrientation(), len*d_showAxisSize.getValue());
    }

    if(d_showCenterOfGravity.getValue())
    {
        gravityCenter /= x.size();
        const sofa::defaulttype::Vec4f color(1.0,1.0,0.0,1.0);

        vparams->drawTool()->drawCross(gravityCenter, d_showAxisSize.getValue(), color);
    }
}

template<class RigidType, class RigidMass>
template<class P>
SOFA_BASE_MECHANICS_API
void UniformMassRigid<RigidType, RigidMass>::drawImpl(const VisualParams* vparams,
                                                      typename std::enable_if<P::spatial_dimensions != 3, int>::type)
{
    if (!vparams->displayFlags().getShowBehaviorModels())
        return;
    const VecCoord& x =mstate->read(core::ConstVecCoordId::position())->getValue();
    ReadAccessor<Data<vector<int> > > indices = d_indices;
    defaulttype::Vec3d len;

    len[0] = len[1] = sqrt(d_mass.getValue().inertiaMatrix);
    len[2] = 0;

    for (unsigned int i=0; i<indices.size(); i++)
    {
        Quat orient(Vec3d(0,0,1), x[indices[i]].getOrientation());
        Vec3d center; center = x[indices[i]].getCenter();

        vparams->drawTool()->drawFrame(center, orient, len*d_showAxisSize.getValue() );
    }
}

template <class RigidType, class RigidMass>
SOFA_BASE_MECHANICS_API
void  UniformMassRigid<RigidType, RigidMass>::setSrcFilename(const std::string& filename)
{
    d_filenameMass.setValue(filename) ;
}

template <class RigidType, class RigidMass>
SOFA_BASE_MECHANICS_API
const std::string& UniformMassRigid<RigidType, RigidMass>::getSrcFilename(){
    return d_filenameMass.getValue() ;
}

//////////////////////////////////////////// REGISTERING TO FACTORY /////////////////////////////////////////
/// Registering the component
/// see: https://www.sofa-framework.org/community/doc/programming-with-sofa/components-api/the-objectfactory/
/// 1-SOFA_DECL_CLASS(componentName) : Set the class name of the component
/// 2-RegisterObject("description") + .add<> : Register the component
/// 3-.add<>(true) : Set default template
SOFA_DECL_CLASS(UniformMassRigid)

// Register in the Factory
int UniformMassRigidClass = core::RegisterObject("Define the same mass for all the particles")

#ifdef SOFA_WITH_DOUBLE
          .add< UniformMassRigid<Rigid3dTypes,Rigid3dMass> >()
   //       .add< UniformMassRigid<Rigid2dTypes,Rigid2dMass> >()
#endif
#ifdef SOFA_WITH_FLOAT
          .add< UniformMassRigid<Rigid3fTypes,Rigid3fMass> >()
 //         .add< UniformMassRigid<Rigid2fTypes,Rigid2fMass> >()
#endif
        ;
////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef SOFA_WITH_DOUBLE
template class SOFA_BASE_MECHANICS_API UniformMassRigid<Rigid3dTypes,Rigid3dMass>;
template class SOFA_BASE_MECHANICS_API UniformMassRigid<Rigid2dTypes,Rigid2dMass>;
#endif

#ifdef SOFA_WITH_FLOAT
template class SOFA_BASE_MECHANICS_API UniformMassRigid<Rigid3fTypes,Rigid3fMass>;
//template class SOFA_BASE_MECHANICS_API UniformMassRigid<Rigid2fTypes,Rigid2fMass>;
#endif

} // namespace mass

} // namespace component

} // namespace sofa

