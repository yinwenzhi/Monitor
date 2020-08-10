#ifndef CONFIG_H
#define CONFIG_H
#include <iostream>
#include <string>
#include "common_include.h"

namespace Monitor
{
    class Config
    {
    private:
        static std::shared_ptr<Config> config_;
        cv::FileStorage file_;
        std::string filename_;

        Config () {} // private constructor makes a singleton
    public:
        ~Config();  // close the file when deconstructing

        // set a new config file
        static void setParameterFile( const std::string& filename );

        static std::shared_ptr<Config> getInstance();

        // access the parameter values
        template< typename T >
        static T get( const std::string& key )
        {
            return T( Config::config_->file_[key] );
        }
    };
}// namespace Monitor
#endif //CONFIG_H
