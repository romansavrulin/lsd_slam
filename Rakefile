
$:.unshift File.dirname(__FILE__) + "/.rb"
require 'pathname'
require 'docker'
require 'conan'

## Set defaults
@cmake_opts = ['-DBUILD_UNIT_TESTS:BOOL=True']
@conan_opts = {}
@conan_settings = {}
@conan_scopes = { build_tests: 'True' }
load 'config.rb' if FileTest::exists? 'config.rb'

build_root = ENV['BUILD_ROOT'] || "build"


task :default => "debug:test"

builds = %w( Release Debug Debug_NoGUI )
builds.each do |build|

  deps_touchfile = '.DEPS_MADE'

  namespace build.downcase do

    build_type = build.split('_').first
    @cmake_opts << '-DBUILD_GUI:bool=False' if( build =~ /NoGUI/  )

    cmake_args = %W( -DCMAKE_BUILD_TYPE:string=#{build_type}
                  #{ENV['CMAKE_FLAGS']}
                  #{@cmake_opts.join(' ')}
                  -DEXTERNAL_PROJECT_PARALLELISM:string=0 )

    build_dir = [build_root, build].join('-')

    desc "Make lsd_slam for #{build}"
    task :build  do
      mkdir build_dir unless FileTest.directory? build_dir
      chdir build_dir do
        sh "cmake % s .." % cmake_args.join(' ')
        sh "make deps && touch #{deps_touchfile}" unless File.readable? deps_touchfile
        sh "make"
      end
    end

    ## Force make deps
    desc "Force rebuild of the dependencies for #{build}"
    task :deps  do
      chdir build_dir do
        sh "cmake", *cmake_args
        FileUtils.rm deps_touchfile
        sh "make deps && touch #{deps_touchfile}"
      end
    end

    task :clean  do
      chdir build_dir do
        sh "make clean"
      end
    end

    desc "Run all tests"
    task :test => [ :build, "test:unit" ]

    namespace :test do
      desc "Unit tests for #{build}"
      task :unit do
        chdir build_dir do
          sh "make unit_test"
        end
      end
    end

  end

end

# Conan builds default to no GUI
ConanTasks.new( builds: %w( Release Debug Debug_GUI ), opts: @conan_opts, settings: @conan_settings, scopes: @conan_scopes )

DockerTasks.new( builds: builds )

#
# Platform-specific tasks for installing dependencies
#
namespace :dependencies do

  desc "Install non-GUI dependencies for Ubuntu Xenial"
  task :xenial => :trusty
  namespace :xenial do
    desc "Install GUI and non-GUI dependencies for Ubuntu Xenial"
    task :gui=> 'dependencies:trusty:gui'
  end

  desc "Install non-GUI dependencies for Ubuntu trusty"
  task :trusty do
    sh "sudo apt-get update &&
        sudo apt-get install -y cmake \
          libopencv-dev libboost-all-dev libeigen3-dev \
          libgomp1 libsuitesparse-dev git \
          autoconf libtool"
  end

  namespace :trusty do
    desc "Install GUI and non-GUI dependencies for Ubuntu trust"
    task :gui => 'dependencies:trusty' do
      sh "sudo apt-get install -y \
        		libglew-dev libglm-dev freeglut3-dev"
    end
  end

  desc "Install non-GUI depenencies on OSX using Brew"
  task :osx_brew do
    sh "brew update"
    sh "brew tap homebrew/science"
    sh "brew install homebrew/science/opencv homebrew/science/suitesparse \
            eigen"
  end

  namespace :osx_brew do
    desc "Install GUI and non-GUI dependencies on OSX using Brew"
    task :gui => 'dependencies:osx_brew' do
      sh "brew install glew glm" # freeglut"   Deprecate GLUT and use native windowing instead?
    end
  end

  ## Travis-specific depenendcy rules
  namespace :travis do

    task :linux => 'dependencies:trusty'
    namespace :linux do
      task :gui => 'dependencies:trusty:gui'
    end

    task :osx => [:pip_uninstall_numpy, 'dependencies:osx_brew']
    namespace :osx do
      task :gui => [:pip_uninstall_numpy, 'dependencies:osx_brew:gui']
    end

    # This installed version conflicts with the version brought in by OpenCV in Homebrew?
    task :pip_uninstall_numpy do
      sh "pip uninstall -y numpy"
    end

  end



end
