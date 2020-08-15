
#include "../../include/Monitor/Config.h"

namespace Monitor
{
    std::shared_ptr<Config> Config::config_ = nullptr;

    void Config::setParameterFile( const std::string& filename )
    {
        if ( config_ == nullptr ){
            config_ = shared_ptr<Config>(new Config);
        }
        config_->filename_ = filename;
        config_->file_ = cv::FileStorage( filename.c_str(), cv::FileStorage::READ );
        if ( config_->file_.isOpened() == false )
        {
            std::cerr<<"parameter file "<<filename<<" does not exist."<<std::endl;
            config_->file_.release();
            return;
        }
    }

    std::shared_ptr<Config> Config::getInstance(){
        return config_;
    }

    Config::~Config()
    {
        if ( file_.isOpened() ){
            file_.release();
            std::cout << "setting file_ released."<< std::endl;
        }

    }


} //namespace Monitor
