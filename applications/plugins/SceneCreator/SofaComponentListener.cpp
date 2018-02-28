#include "SofaComponentListener.h"
#include <sofa/core/ObjectFactory.h>
#include <sofa/defaulttype/VecTypes.h>

using namespace sofa::core::objectmodel;

namespace sofa
{

SOFA_DECL_CLASS(SofaComponentListener)

int SofaComponentListenerClass = sofa::core::RegisterObject("SofaComponentListener component")
.add< SofaComponentListener >()
;


SofaComponentListener::SofaComponentListener()
    : InputComponentPath(initData(&InputComponentPath, "InputComponentPath", "Path of Sofa component to listen."))
    , m_sofaObject(NULL)
{
	
}

SofaComponentListener::~SofaComponentListener()
{

}

void SofaComponentListener::init()
{
    std::cout << "SofaComponentListener::init()" << std::endl;

    if (m_sofaObject == NULL)
    {
        const std::string& path = InputComponentPath.getValue();
        if (path.size() > 0)
        {
            this->getContext()->get(m_sofaObject, path);
        }
    }

    /*
    if (m_sofaObject != NULL)
    {
        std::cout << "SofaComponentListener m_sofaObject creation success" << std::endl;

        const VecData& datas = m_sofaObject->getDataFields();
        for (int i = 0; i < datas.size(); i++)
        {
            BaseData* data = datas[i];
            std::cout << data->getName() << " - " << data->getValueTypeString() << " | " << data->getValueString() << std::endl;
        }
    }
    */
}

sofa::core::objectmodel::BaseObject* SofaComponentListener::getSofaComponent()
{
    if (m_sofaObject == NULL)
    {
        const std::string& path = InputComponentPath.getValue();
        if (path.size() > 0)
        {
            this->getContext()->get(m_sofaObject, path);
        }
    }

    return m_sofaObject;
}

void SofaComponentListener::reinit()
{
    std::cout << "SofaComponentListener::reinit()" << std::endl;
}

} //namespace sofa
