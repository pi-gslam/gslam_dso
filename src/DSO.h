#ifndef DSO_H
#define DSO_H
#include <iostream>
#include <Eigen/Dense>
#include <GSLAM/core/GSLAM.h>

namespace dso
{
    class FullSystem;
    class Undistort;
}

class WarperGSLAM;
class ImageFolderReader;

namespace GSLAM {


class DSO
{
public:
    DSO(Svar config);
    virtual ~DSO();

    virtual std::string type()const{return "DSO";}
    virtual bool valid()const;

    virtual bool track(FramePtr& frame);

    virtual void draw();

    virtual void call(const std::string& command,void* arg=NULL);

    virtual bool isDrawable()const{return true;}

    std::shared_ptr<dso::FullSystem>   system;
    std::shared_ptr<WarperGSLAM>       warper;

    std::shared_ptr<ImageFolderReader> reader;

    bool                    shouldUndistort;
    Subscriber subDataset;
    Publisher  pubMap,pubFrame;
};

}
#endif // DSO_H
