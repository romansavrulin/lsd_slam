#pragma once

#include <g3log/logworker.hpp>

namespace lsd_slam {

std::unique_ptr<g3::LogWorker> initializeG3Log( const std::string &appName );

void logBanner( void );

}
