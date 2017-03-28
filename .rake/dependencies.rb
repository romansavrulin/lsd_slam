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
