#ifndef SOFACOMPONENTLISTENER_H
#define SOFACOMPONENTLISTENER_H

#include <SceneCreator/config.h>
#include <sofa/core/objectmodel/BaseObject.h>

namespace sofa
{

class SOFA_SCENECREATOR_API SofaComponentListener : public virtual sofa::core::objectmodel::BaseObject
{
public:
    SOFA_CLASS(SofaComponentListener, sofa::core::objectmodel::BaseObject);

protected:
    SofaComponentListener();
    virtual ~SofaComponentListener();

public:
    void init();
    void reinit();

    sofa::core::objectmodel::BaseObject* getSofaComponent();


    Data<std::string> InputComponentPath;

protected:
    sofa::core::objectmodel::BaseObject* m_sofaObject;

};

} // namespace sofa

#endif //SOFACOMPONENTLISTENER_H
