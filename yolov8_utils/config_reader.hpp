#ifndef __CONFIG_READER__
#define __CONFIG_READER_

#include <iostream>
#include <string>


#include "utils.hpp"
#include "dla_config.hpp"

#if defined (SPDLOG)
#include "logger.hpp"
#endif

#define CONFIG_ENABLE 1
#define CONFIG_DISABLE 0

class TrackerConfigReader
{
  public:
    TrackerConfigReader();
    ~TrackerConfigReader();

    bool read(std::string configPath);
    void getConfig(Config_S *config);
    Config_S* getConfig();

    Config_S* m_config;

  private:
    // static const auto m_logger = spdlog::stdout_color_mt("ADAS_ConfigReader");

};


#endif

