/******************************************************************************
*                 SOFA, Simulation Open-Framework Architecture                *
*                    (c) 2006 INRIA, USTL, UJF, CNRS, MGH                     *
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
#ifndef CPUSORTWITHCUDA_H
#define CPUSORTWITHCUDA_H

#include "../../cuda/mycuda.h"
#include <vector>




template <class T>
class CPUSortWithCuda
{
private:

    struct Element
    {
        int key;
        T value;
    };

    typedef std::vector< Element > Vectorsort;

public:
    static void sort(void * keys,void * values,int numElements)
    {
        T* valueVector = new T[numElements];
        int *keyVector = new int[numElements];
        Vectorsort vecsort;

        sofa::gpu::cuda::mycudaMemcpyDeviceToHost(valueVector,values,numElements*sizeof(T),0);
        sofa::gpu::cuda::mycudaMemcpyDeviceToHost(keyVector,keys,numElements*sizeof(int),0);

        for(int i=0; i<numElements; i++)
        {
            //		float *t = (float*)(valueVector+i);
            //		std::cout << "#" << keyVector[i] << " " << t[0] << "." << t[1] << "." << t[2] << "\n";
            Element e;
            e.key = keyVector[i];
            e.value = valueVector[i];
            vecsort.insert(vecsort.end(),e);
        }

        std::sort( vecsort.begin(), vecsort.end(), compare);

//	std::cout << "-----------------\n";
        for (unsigned int j=0; j<vecsort.size(); j++)
        {
            //		float *t = (float*)(&(vecsort[j].value));
            //		std::cout << " " << vecsort[j].key <<":"<< t[0] << "#" << t[1] << "#" << t[2] << "\n";
            keyVector[j] = vecsort[j].key;
            valueVector[j] = vecsort[j].value;
        }

        sofa::gpu::cuda::mycudaMemcpyHostToDevice(values,valueVector,numElements*sizeof(T),0);
        sofa::gpu::cuda::mycudaMemcpyHostToDevice(keys,keyVector,numElements*sizeof(int),0);
    }

private:
    static bool compare(Element e1,Element e2) {return (e1.key<e2.key);}
};

#endif // CPUSORTWITHCUDA_H
