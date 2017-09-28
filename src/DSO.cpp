#include "DSO.h"

#include <GSLAM/core/VecParament.h>
#include <GSLAM/core/Undistorter.h>

#include "FullSystem/FullSystem.h"
#include "IOWrapper/WarperGSLAM.h"
#include "util/ImageAndExposure.h"
#include "util/settings.h"
#include "util/globalCalib.h"
#include "util/MinimalImage.h"
#include "util/Undistort.h"
#include "util/DatasetReader.h"

using namespace std;
using namespace dso;

namespace GSLAM
{

pi::SE3d fromSophus(const dso::SE3& se3)
{
    pi::SE3d result;
    Eigen::Vector3d trans=se3.translation();
    result.get_translation()=*(pi::Point3d*)&trans;

    Eigen::Matrix3d rot =se3.inverse().rotationMatrix();
    result.get_rotation().fromMatrixUnsafe(rot);

    return result;
}

dso::SE3 ToSophus(pi::SE3<double> se3_zy)
{
    Eigen::Matrix3d eigen_rot;
    se3_zy.get_rotation().inv().getMatrixUnsafe(eigen_rot);
    Eigen::Vector3d eigen_trans=*((Eigen::Vector3d*)&se3_zy.get_translation());

    return Sophus::SE3(eigen_rot,eigen_trans);
}

void settingsDefault(int preset)
{
    float playbackSpeed=0,preload;
    printf("\n=============== PRESET Settings: ===============\n");
    if(preset == 0 || preset == 1)
    {
        printf("DEFAULT settings:\n"
                "- %s real-time enforcing\n"
                "- 2000 active points\n"
                "- 5-7 active frames\n"
                "- 1-6 LM iteration each KF\n"
                "- original image resolution\n", preset==0 ? "no " : "1x");

        playbackSpeed = (preset==0 ? 0 : 1);
        preload = preset==1;
        setting_desiredImmatureDensity = 1500;
        setting_desiredPointDensity = 2000;
        setting_minFrames = 5;
        setting_maxFrames = 7;
        setting_maxOptIterations=6;
        setting_minOptIterations=1;

        setting_logStuff = false;
    }

    if(preset == 2 || preset == 3)
    {
        printf("FAST settings:\n"
                "- %s real-time enforcing\n"
                "- 800 active points\n"
                "- 4-6 active frames\n"
                "- 1-4 LM iteration each KF\n"
                "- 424 x 320 image resolution\n", preset==0 ? "no " : "5x");

        playbackSpeed = (preset==2 ? 0 : 5);
        preload = preset==3;
        setting_desiredImmatureDensity = 600;
        setting_desiredPointDensity = 800;
        setting_minFrames = 4;
        setting_maxFrames = 6;
        setting_maxOptIterations=4;
        setting_minOptIterations=1;

        benchmarkSetting_width = 424;
        benchmarkSetting_height = 320;

        setting_logStuff = false;
    }

    printf("==============================================\n");
}



DSO::DSO()
    :shouldUndistort(false)
{
    settingsDefault(svar.GetInt("DSO.Preset",3));

    setting_photometricCalibration = 0;
    setting_affineOptModeA = 0; //-1: fix. >=0: optimize (with prior, if > 0).
    setting_affineOptModeB = 0; //-1: fix. >=0: optimize (with prior, if > 0).

//    string source=svar.GetString("DSO.Dataset","");
//    string calib=svar.GetString("DSO.CalibFileName","");
//    string gammaCalib=svar.GetString("DSO.GammaFile","");
//    string vignette=svar.GetString("DSO.VigFile","");
//    reader=SPtr<ImageFolderReader>(new ImageFolderReader(source,calib, gammaCalib, vignette));
//    reader->setGlobalCalibration();
}

DSO::~DSO()
{
    if(system.get())
    {
        system->blockUntilMappingIsFinished();
        system->outputWrapper=NULL;
        system.reset();
    }
    warper.reset();
}

bool DSO::valid()const
{
    return true;
}

bool DSO::track(FramePtr& frame)
{
    using namespace dso;
    if(frame->cameraNum()<1) return false;

    if(!system.get())//first frame
    {
        // calibration data should be setted before system loaded!
        GSLAM::Camera camera=frame->getCamera(1);
        if(camera.isValid()&&camera.info()!=frame->getCamera().info()) shouldUndistort=true;
        else camera=frame->getCamera();

        VecParament<double> para=camera.getParameters();
        Eigen::Matrix3f K;


        {
            K.setIdentity();
            K(0,0)=para[2];
            K(1,1)=para[3];
            K(0,2)=para[4];
            K(1,2)=para[5];
        }
        cout<<"K="<<K<<endl;

        setGlobalCalib(camera.width(),camera.height(),K);

        system=SPtr<dso::FullSystem>(new dso::FullSystem());
        warper=SPtr<WarperGSLAM>(new WarperGSLAM(camera.width(),camera.height()));

        if("VideoFrameMonoWithExposure"==frame->type())
        {
            GSLAM::GImage G=frame->getImage(1);
            system->setGammaFunction((float*)G.data);
        }
    }

    GSLAM::GImage gimg=frame->getImage();

    if(reader.get()&&shouldUndistort)
    {

        cerr<<"Please do undistort outside first!\n";
        return false;

        MinimalImage<uchar> minimal(gimg.cols,gimg.rows,gimg.data);

        SPtr<dso::ImageAndExposure> img(reader->undistort->undistort<unsigned char>(&minimal,1));

        system->addActiveFrame(img.get(),frame->id());
    }
    else
    {
        float exposure_time=0;
        if("VideoFrameMonoWithExposure"==frame->type())
        {
            frame->call("getExposure",&exposure_time);
        }

        if(gimg.type()==GSLAM::GImageType<uchar,3>::Type)
        {
            GSLAM::GImage tmp(gimg.cols,gimg.rows,GSLAM::GImageType<uchar,1>::Type);
            for(int i=0,iend=gimg.total();i<iend;i++)
                tmp.data[i]=gimg.data[i*3];
            gimg=tmp;
        }
        else if(gimg.type()==GSLAM::GImageType<uchar,4>::Type)
        {
            GSLAM::GImage tmp(gimg.cols,gimg.rows,GSLAM::GImageType<uchar,1>::Type);
            for(int i=0,iend=gimg.total();i<iend;i++)
                tmp.data[i]=gimg.data[i*4];
            gimg=tmp;
        }

        if(frame->getCamera(1).isValid())
        {
            static GSLAM::Undistorter undis(frame->getCamera(0),frame->getCamera(1));
            GSLAM::GImage tmp;
            undis.undistortFast(gimg,tmp);
            gimg=tmp;
        }

        SPtr<dso::ImageAndExposure> img(new dso::ImageAndExposure(gimg.cols,gimg.rows,frame->_timestamp));
        img->exposure_time=exposure_time;


        if(gimg.type()==GSLAM::GImageType<uchar,1>::Type)
            for(int i=0,iend=gimg.total();i<iend;i++)
                img->image[i]=gimg.data[i];
        else
        {
            cerr<<"image are not u8 gray!\n";
        }


        system->addActiveFrame(img.get(),frame->id());
    }

    if(system->initFailed || dso::setting_fullResetRequested)
    {
        if(frame->id() < 250 || dso::setting_fullResetRequested)
        {
            printf("RESETTING!\n");
            system.reset();
            if(warper.get())
                warper->reset();
            dso::setting_fullResetRequested=false;
            return false;
        }
    }

    if(system->isLost)
    {
        printf("LOST!!\n");
        return false;
    }

    frame->setPose(fromSophus(warper->lastPose));
    return true;

}

void DSO::draw()
{
    if(system.get())
        system->outputWrapper=warper.get();

    if(warper)
        warper->draw();
}

void DSO::call(const std::string& command,void* arg)
{

}


}


USE_GSLAM_PLUGIN(GSLAM::DSO);
