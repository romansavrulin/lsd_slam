
require 'pathname'

# Rake support files are stored in the .rake subdirectory
$:.unshift File.dirname(__FILE__) + "/.rake"
require 'docker'
require 'conan'
require 'dependencies'
require 'benchmark'
require 'build'
require 'build_tasks'

## Set defaults
@cmake_opts = ['-DBUILD_UNIT_TESTS:BOOL=True']
@conan_opts = {}
@conan_settings = {}
@conan_scopes = { build_tests: 'True' }

##
@coverity_email = ENV['LSDSLAM_COVERITY_EMAIL']
@coverity_token = ENV['LSDSLAM_COVERITY_TOKEN']

@build_parallelism = nil

## Any of the above configuration variables can be overridden in a config.rb
## file
load 'config.rb' if FileTest::exists? 'config.rb'


## Builds occur in directories "#{BUILD_ROOT}-#{build_type}"
## e.g. build-debug/, build-release/
build_root = ENV['BUILD_ROOT'] || "build"

cmake = CMake.new

newBuilds = [ Build.new( "Debug", cmake: cmake  ),
              Build.new( "Debug_NoGUI", gui: false, cmake: cmake ),
              Build.new( "Release", cmake: cmake  )
            ]
BuildTasks.new( newBuilds )

task :default => "debug:test"


# Conan builds default to no GUI, so Debug_GUI needs to be explicitly included
ConanTasks.new( builds: %w( Release Debug Debug_GUI ), opts: @conan_opts, settings: @conan_settings, scopes: @conan_scopes )
DockerTasks.new( builds: %w( Release Debug Debug_GUI ) )
BenchmarkTasks.new( newBuilds )
