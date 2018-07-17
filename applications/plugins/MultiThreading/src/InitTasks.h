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
#ifndef InitTasks_h__
#define InitTasks_h__

#include "TaskScheduler.h"

//#include <sofa/helper/system/atomic.h>

namespace sofa
{
namespace simulation
{

	using namespace sofa;



	class InitPerThreadDataTask : public Task
	{

	public:

		//InitPerThreadDataTask(volatile long* atomicCounter, boost::mutex* mutex, TaskStatus* pStatus );
		InitPerThreadDataTask(std::atomic<int>* atomicCounter, std::mutex* mutex, Task::Status* pStatus );
		
		virtual ~InitPerThreadDataTask();

		virtual bool run(WorkerThread* );

	private:

		std::mutex*	 IdFactorygetIDMutex;
		std::atomic<int>* _atomicCounter;
	};



	class InitOGLcontextTask : public Task
	{
	public:
		InitOGLcontextTask::InitOGLcontextTask(HDC& glDevice, HGLRC& workerThreadContext, std::atomic<int>* atomicCounter, std::mutex* mutex, Task::Status* pStatus);

		InitOGLcontextTask::~InitOGLcontextTask();

		bool run(sofa::simulation::WorkerThread*);

	private:

		HDC & _glDevice;
		HGLRC& _workerThreadContext;
		std::mutex*	 IdFactorygetIDMutex;
		std::atomic<int>* _atomicCounter;
	};


	class DeleteOGLcontextTask : public sofa::simulation::Task
	{
	public:
		DeleteOGLcontextTask::DeleteOGLcontextTask(std::atomic<int>* atomicCounter, std::mutex* mutex, Task::Status* pStatus);

		DeleteOGLcontextTask::~DeleteOGLcontextTask();

		bool run(sofa::simulation::WorkerThread*);

	private:
		std::mutex*	 IdFactorygetIDMutex;
		std::atomic<int>* _atomicCounter;
	};


	//fix and prefer using the global runThreadSpecificTask
	SOFA_MULTITHREADING_PLUGIN_API void initThreadLocalData();

	SOFA_MULTITHREADING_PLUGIN_API void initOGLcontext();

	SOFA_MULTITHREADING_PLUGIN_API void deleteOGLcontext();


} // namespace simulation

} // namespace sofa

#endif // InitTasks_h__
