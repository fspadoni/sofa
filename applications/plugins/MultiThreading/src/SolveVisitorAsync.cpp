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
#include "SolveVisitorAsync.h"
#include <sofa/core/behavior/BaseMechanicalState.h>

#include <sofa/helper/AdvancedTimer.h>
#include <MultiThreading/src/TaskScheduler.h>
#include <MultiThreading/src/TaskSchedulerProfiler.h>

namespace sofa
{

namespace simulation
{

    DEFINE_TASK_SCHEDULER_PROFILER(SolveVisitorAsyncTask);

	class SolveVisitorAsyncTask : public simulation::Task
	{
	public:
		SolveVisitorAsyncTask(
			SolveVisitorAsync* solveVisitorAsync, 
			simulation::Node* node, 
			Node::Sequence<sofa::core::behavior::OdeSolver>* solver, 
			Task::Status* status)
			: Task(status)
			, _solveVisitorAsync(solveVisitorAsync)
			, _node(node)
			,_solver(solver)
		{}

		virtual ~SolveVisitorAsyncTask()
        { 
            //delete this;
        }

		virtual Task::Memory run(simulation::WorkerThread*)
		{
            //TASK_SCHEDULER_PROFILER(SolveVisitorAsyncTask);

            auto start = std::chrono::high_resolution_clock::now();

			//void (SolveVisitorAsync::*)(VContext*, Object*) 
			//auto processSolver_func = std::bind(&_solveVisitorAsync->processSolver, _node, _solver);//  _solveVisitorAsync->processSolver(_node, _solver);
			//_solveVisitorAsync->for_each(_solveVisitorAsync, _node, _solver, processSolver_func);
			_solveVisitorAsync->processSolver(_node, (*_solver)[0]);

            auto end = std::chrono::high_resolution_clock::now();
            _millis = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
			return Task::Memory::Delete;
		}

	private:

		SolveVisitorAsync* _solveVisitorAsync;
		simulation::Node* _node;
		Node::Sequence<sofa::core::behavior::OdeSolver>* _solver;
        long long _millis;
	};


	void SolveVisitorAsync::processSolver(simulation::Node* node, core::behavior::OdeSolver* s)
	{
		//sofa::helper::AdvancedTimer::stepBegin("Mechanical",node);
		s->solve(core::ExecParams::defaultInstance(), _dt, _x, _v);

		//sofa::helper::AdvancedTimer::stepEnd("Mechanical",node);
	}

	Visitor::Result SolveVisitorAsync::processNodeTopDown(simulation::Node* node)
	{
		if (! node->solver.empty())
		{

			//TaskLockFree::Status status;

			simulation::WorkerThread* thread = simulation::WorkerThread::getCurrent();

			SolveVisitorAsyncTask* task = new SolveVisitorAsyncTask(this, node, &node->solver, _status);
			thread->addTask(task);
			//for_each(this, node, node->solver, &SolveVisitorAsync::processSolver);

			return RESULT_PRUNE;
		}
		else
			return RESULT_CONTINUE;
	}

} // namespace simulation

} // namespace sofa

