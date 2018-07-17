/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, development version     *
*                (c) 2006-2018 INRIA, USTL, UJF, CNRS, MGH                    *
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
#ifndef sofa_FreeMotionAnimaitionLoop_h__
#define sofa_FreeMotionAnimaitionLoop_h__

#include <MultiThreading/config.h>

#include <sofa/simulation/CollisionAnimationLoop.h>
#include <SofaConstraint/LCPConstraintSolver.h>

#include <Multithreading/src/Tasks.h>

namespace sofa
{

namespace component
{

namespace animationloop
{

class SOFA_MULTITHREADING_PLUGIN_API FreeMotionAnimationLoopAsync : public sofa::simulation::CollisionAnimationLoop
{
public:
    typedef sofa::simulation::CollisionAnimationLoop Inherit;

    SOFA_CLASS(FreeMotionAnimationLoopAsync, sofa::simulation::CollisionAnimationLoop);

protected:
	FreeMotionAnimationLoopAsync(simulation::Node* gnode);
    virtual ~FreeMotionAnimationLoopAsync();
public:
    virtual void step (const sofa::core::ExecParams* params, SReal dt) override;

    virtual void init() override;

	/// Initialization method called at graph creation and modification, during bottom-up traversal.
	virtual void bwdInit();

	/// Update method called when variables used in precomputation are modified.
	virtual void reinit();

	virtual void cleanup();

    virtual void parse ( sofa::core::objectmodel::BaseObjectDescription* arg ) override;

    /// Construction method called by ObjectFactory.
    template<class T>
    static typename T::SPtr create(T*, BaseContext* context, BaseObjectDescription* arg)
    {
        simulation::Node* gnode = dynamic_cast<simulation::Node*>(context);
        typename T::SPtr obj = sofa::core::objectmodel::New<T>(gnode);
        if (context) context->addObject(obj);
        if (arg) obj->parse(arg);
        return obj;
    }


    Data<bool> displayTime;

    Data<bool> m_solveVelocityConstraintFirst; ///< solve separately velocity constraint violations before position constraint violations

	Data<int> threadNumber; ///< number of thread

private:
	// thread storage initialization
	void initThreadLocalData();

protected :

    sofa::core::behavior::ConstraintSolver *constraintSolver;
    component::constraintset::LCPConstraintSolver::SPtr defaultSolver;

	//simulation::TaskLockFree::Status _solverTaskStatus;

	int _nbThread;
};

} // namespace animationloop

} // namespace component

} // namespace sofa

#endif /* sofa_FreeMotionAnimaitionLoop_h__ */
