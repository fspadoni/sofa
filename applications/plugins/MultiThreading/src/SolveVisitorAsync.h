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
#ifndef Sofa_SolveVisitorAsync_h__
#define Sofa_SolveVisitorAsync_h__


#include <MultiThreading/src/VisitorAsync.h>
#include <sofa/core/behavior/OdeSolver.h>


namespace sofa
{

namespace simulation
{

/** 
 * Used by the animation loop: send the solve signal to the others solvers
 */
class SOFA_MULTITHREADING_PLUGIN_API SolveVisitorAsync : public VisitorAsync
{
public:
	SolveVisitorAsync(const sofa::core::ExecParams* params, Task::Status* status, SReal dt)
		: VisitorAsync(params, status)
		, _dt(dt)
		, _x(core::VecCoordId::position())
		, _v(core::VecDerivId::velocity())
	{}

	SolveVisitorAsync(const sofa::core::ExecParams* params, Task::Status* status, SReal dt, bool free)
		: VisitorAsync(params, status)
		, _dt(dt)
	{
        if(free){
            _x = core::VecCoordId::freePosition();
            _v = core::VecDerivId::freeVelocity();
        }
        else{
            _x = core::VecCoordId::position();
            _v = core::VecDerivId::velocity();
        }
    }

	SolveVisitorAsync(const sofa::core::ExecParams* params, Task::Status* status, SReal dt, sofa::core::MultiVecCoordId X,sofa::core::MultiVecDerivId V)
		: VisitorAsync(params, status)
		, _dt(dt)
		, _x(X)
		, _v(V)
	{}

    virtual void processSolver(simulation::Node* node, core::behavior::OdeSolver* b);
    virtual Result processNodeTopDown(simulation::Node* node);

    /// Specify whether this action can be parallelized.
    virtual bool isThreadSafe() const { return true; }

    /// Return a category name for this action.
    /// Only used for debugging / profiling purposes
    virtual const char* getCategoryName() const { return "behavior update position"; }
    virtual const char* getClassName() const { return "SolveVisitorAsync"; }

    void setDt(SReal dt) {_dt = dt;}
    SReal getDt() {return _dt;}

protected:

    SReal _dt;
    sofa::core::MultiVecCoordId _x;
    sofa::core::MultiVecDerivId _v;
};

} // namespace simulation

} // namespace sofa

#endif // Sofa_SolveVisitorAsync_h__
