#include <iostream>
#include <string.h>
#include <type_traits>

#include <sofa/defaulttype/BaseMatrix.h>
using sofa::defaulttype::BaseMatrix ;

#include <SofaBaseMechanics/UniformMassVec.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/topology/Topology.h>
#include <sofa/core/objectmodel/Context.h>
#include <sofa/helper/accessor.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/DataTypeInfo.h>
#include <SofaBaseMechanics/AddMToMatrixFunctor.h>
#include <sofa/simulation/Simulation.h>

#include <SofaBaseMechanics/UniformMass.inl>
using sofa::component::mass::UniformMass ;

#include <sofa/defaulttype/VecTypes.h>
using sofa::component::mass::Vec3d ;

#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/core/ObjectFactory.h>

#include <sofa/helper/system/FileRepository.h>
using sofa::helper::system::DataRepository ;

#include <sofa/helper/system/Locale.h>

#include <sstream>
using std::string ;
using std::ostringstream ;


using namespace sofa::defaulttype;

namespace sofa
{

namespace component
{

namespace mass
{

template <class DataTypes, class MassType>
UniformMassVec<DataTypes, MassType>::UniformMassVec() : UniformMass<DataTypes, MassType>(){

}

template <class DataTypes, class MassType>
UniformMassVec<DataTypes, MassType>::~UniformMassVec(){}

template <class PunctualType, class PunctualMass> SOFA_BASE_MECHANICS_API
void UniformMassVec<PunctualType, PunctualMass>::reinit()
{
    std::cout << "REINIT PUNCTUAL ..." << std::endl ;
    WriteAccessor<Data<vector<int> > > indices = d_indices;
    m_doesTopoChangeAffect = false;

    if(mstate==NULL){
        msg_warning(this) << "Missing mechanical state. \n"
                             "UniformMass need to be used with an object also having a MechanicalState. \n"
                             "To remove this warning: add a <MechanicalObject/> to the parent node of the one \n"
                             " containing this <UniformMass/>";
        return;
    }

    //If localRange is set, update indices
    if (d_localRange.getValue()[0] >= 0
            && d_localRange.getValue()[1] > 0
            && d_localRange.getValue()[1] + 1 < (int)mstate->getSize())
    {
        indices.clear();
        for(int i=d_localRange.getValue()[0]; i<=d_localRange.getValue()[1]; i++)
            indices.push_back(i);
    }

    //If no given indices
    if(indices.size()==0)
    {
        indices.clear();
        for(int i=0; i<(int)mstate->getSize(); i++)
            indices.push_back(i);
        m_doesTopoChangeAffect = true;
    }

    //Update mass and totalMass
    if (d_totalMass.getValue() > 0)
    {
        PunctualMass *m = d_mass.beginEdit();
        *m = ( (Real) d_totalMass.getValue() / indices.size() );
        d_mass.endEdit();
    }
    else
        d_totalMass.setValue ( indices.size() * (Real)d_mass.getValue() );
}

template <class PunctualType, class PunctualMass>
SOFA_BASE_MECHANICS_API
void UniformMassVec<PunctualType, PunctualMass>::draw(const core::visual::VisualParams* vparams)
{
    Inherit1::draw(vparams) ;
}

template <> SOFA_BASE_MECHANICS_API
void UniformMassVec<Vec6dTypes, double>::draw(const core::visual::VisualParams* vparams)
{
    if (!vparams->displayFlags().getShowBehaviorModels())
        return;
    const VecCoord& x =mstate->read(core::ConstVecCoordId::position())->getValue();
    const VecCoord& x0 = mstate->read(core::ConstVecCoordId::restPosition())->getValue();
    ReadAccessor<Data<vector<int> > > indices = d_indices;

    Mat3x3d R; R.identity();

    std::vector<Vector3> vertices;
    std::vector<sofa::defaulttype::Vec4f> colors;

    const sofa::defaulttype::Vec4f red(1.0,0.0,0.0,1.0);
    const sofa::defaulttype::Vec4f green(0.0,1.0,0.0,1.0);
    const sofa::defaulttype::Vec4f blue(0.0,0.0,1.0,1.0);

    sofa::defaulttype::Vec4f colorSet[3];
    colorSet[0] = red;
    colorSet[1] = green;
    colorSet[2] = blue;

    for (unsigned int i=0; i<indices.size(); i++)
    {
        defaulttype::Vec3d len(1,1,1);
        int a = (i<indices.size()-1)?i : i-1;
        int b = a+1;
        defaulttype::Vec3d dp; dp = x0[b]-x0[a];
        defaulttype::Vec3d p; p = x[indices[i]];
        len[0] = dp.norm();
        len[1] = len[0];
        len[2] = len[0];

        Quatd q=Quatd::fromEuler(x[indices[i]][3], x[indices[i]][4], x[indices[i]][5]) ;
        Mat3x3d m;
        q.toMatrix(m);
        R = R * m;

        for(unsigned int j=0 ; j<3 ; j++)
        {
            vertices.push_back(p);
            vertices.push_back(p + R.col(j)*len[j]);
            colors.push_back(colorSet[j]);
            colors.push_back(colorSet[j]);;
        }
    }

    vparams->drawTool()->drawLines(vertices, 1, colors);
}

template <class PunctualType, class PunctualMass>
SOFA_BASE_MECHANICS_API
void UniformMassVec<PunctualType, PunctualMass>::addMDxToVector(defaulttype::BaseVector *resVect,
                                                                const VecDeriv* dx,
                                                                SReal mFact,
                                                                unsigned int& offset)
{
    unsigned int derivDim = (unsigned)Deriv::size();
    double m = d_mass.getValue();

    ReadAccessor<Data<vector<int> > > indices = d_indices;

    const double* g = getContext()->getGravity().ptr();

    for (unsigned int i=0; i<indices.size(); i++)
        for (unsigned int j=0; j<derivDim; j++)
        {
            if (dx != NULL)
                resVect->add(offset + indices[i] * derivDim + j, mFact * m * g[j] * (*dx)[indices[i]][0]);
            else
                resVect->add(offset + indices[i] * derivDim + j, mFact * m * g[j]);
        }
}

template <class PunctualType, class PunctualMass>
SReal UniformMassVec<PunctualType, PunctualMass>::getPotentialEnergy ( const MechanicalParams* params,
                                                                       const DataVecCoord& d_x  ) const
{
    SOFA_UNUSED(params);
    ReadAccessor<DataVecCoord> x = d_x;
    ReadAccessor<Data<vector<int> > > indices = d_indices;

    SReal e = 0;
    const PunctualMass& m = d_mass.getValue();

    Vec3d g( getContext()->getGravity());
    Deriv gravity;
    PunctualType::set(gravity, g[0], g[1], g[2]);

    Deriv mg = gravity * m;

    for ( unsigned int i=0; i<indices.size(); i++ )
        e -= mg*x[indices[i]];

    return e;
}


template <class PunctualType, class PunctualMass>
SOFA_BASE_MECHANICS_API
Vector6 UniformMassVec<PunctualType, PunctualMass>::getMomentum  ( const MechanicalParams*p,
                                                                   const DataVecCoord& d_x,
                                                                   const DataVecDeriv& d_v ) const
{
    return dirtyHackGetMomentum<PunctualType>(p,d_x,d_v,0) ;
}

template <class PunctualType, class PunctualMass>
template <class P>
Vector6 UniformMassVec<PunctualType, PunctualMass>::dirtyHackGetMomentum ( const MechanicalParams*,
                               const DataVecCoord& d_x,
                               const DataVecDeriv& d_v,
                               typename std::enable_if<P::spatial_dimensions==3, int>::type t) const
{
    std::cout << "DIRTY " << std::endl ;

    helper::ReadAccessor<DataVecDeriv> v = d_v;
    helper::ReadAccessor<DataVecCoord> x = d_x;
    ReadAccessor<Data<vector<int> > > indices = d_indices;

    const double& m = d_mass.getValue();

    defaulttype::Vec6d momentum;

    for ( unsigned int i=0 ; i<indices.size() ; i++ )
    {
        Deriv linearMomentum = m*v[indices[i]];
        for( int j=0 ; j<PunctualType::spatial_dimensions ; ++j ) momentum[j] += linearMomentum[j];

        Deriv angularMomentum = cross( x[indices[i]], linearMomentum );
        for( int j=0 ; j<PunctualType::spatial_dimensions ; ++j ) momentum[3+j] += angularMomentum[j];
    }

    return momentum;
}

template <class PunctualType, class PunctualMass>
template <class P>
Vector6 UniformMassVec<PunctualType, PunctualMass>::dirtyHackGetMomentum ( const MechanicalParams*,
                               const DataVecCoord& d_x,
                               const DataVecDeriv& d_v,
                               typename std::enable_if<P::spatial_dimensions!=3, int>::type t) const
{
    std::cout << "WEIRD" << std::endl ;

    msg_warning(this) << "Invalid Calling to the getMomentum() function. \n"
                         "The current implementation only works for Vec3(df) templates.\n"
                         "The current template is " << templateName(this) << " and thus the result of getMomentum is not valid\n" ;
    return defaulttype::Vec6d();
}


//////////////////////////////////////////// REGISTERING TO FACTORY /////////////////////////////////////////
/// Registering the component
/// see: https://www.sofa-framework.org/community/doc/programming-with-sofa/components-api/the-objectfactory/
/// 1-SOFA_DECL_CLASS(componentName) : Set the class name of the component
/// 2-RegisterObject("description") + .add<> : Register the component
/// 3-.add<>(true) : Set default template
SOFA_DECL_CLASS(UniformMassVec)

// Register in the Factory
int UniformMassVecClass = core::RegisterObject("Define the same mass for all the particles")
#ifdef SOFA_WITH_DOUBLE
        .add< UniformMassVec<Vec3dTypes,double> >()
        .add< UniformMassVec<Vec2dTypes,double> >()
        .add< UniformMassVec<Vec1dTypes,double> >()
        .add< UniformMassVec<Vec6dTypes,double> >()
#endif
#ifdef SOFA_WITH_FLOAT
        .add< UniformMassVec<Vec3fTypes,float> >()
        .add< UniformMassVec<Vec2fTypes,float> >()
        .add< UniformMassVec<Vec1fTypes,float> >()
        .add< UniformMassVec<Vec6fTypes,float> >()
#endif
;
////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef SOFA_WITH_DOUBLE
template class SOFA_BASE_MECHANICS_API UniformMassVec<Vec3dTypes,double>;
//template class SOFA_BASE_MECHANICS_API UniformMassVec<Vec2dTypes,double>;
//template class SOFA_BASE_MECHANICS_API UniformMassVec<Vec1dTypes,double>;
//template class SOFA_BASE_MECHANICS_API UniformMassVec<Vec6dTypes,double>;
#endif

#ifdef SOFA_WITH_FLOAT
template class SOFA_BASE_MECHANICS_API UniformMassVec<Vec3fTypes,float>;
//template class SOFA_BASE_MECHANICS_API UniformMassVec<Vec2fTypes,float>;
//template class SOFA_BASE_MECHANICS_API UniformMassVec<Vec1fTypes,float>;
//template class SOFA_BASE_MECHANICS_API UniformMassVec<Vec6fTypes,float>;
#endif

} // namespace mass

} // namespace component

} // namespace sofa

