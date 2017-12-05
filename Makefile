

xenial_deps:
	sudo apt-get update &&
	sudo apt-get install -y cmake \
		libopencv-dev libboost-all-dev libeigen3-dev \
		libgomp1 libsuitesparse-dev git \
		autoconf libtool

trusty_deps:
	sudo apt-get update &&
	sudo apt-get install -y cmake \
		libopencv-dev libboost-all-dev libeigen3-dev \
		libgomp1 libsuitesparse-dev git \
		autoconf libtool


## Code from Rakefile
# namespace :dependencies do
#
#   desc "Install non-GUI dependencies for Ubuntu Xenial"
#   task :xenial => :trusty
#   namespace :xenial do
#     desc "Install GUI and non-GUI dependencies for Ubuntu Xenial"
#     task :gui=> 'dependencies:trusty:gui'
#   end
#
#   desc "Install non-GUI dependencies for Ubuntu trusty"
#   task :trusty do
#     sh "sudo apt-get update &&
#         sudo apt-get install -y cmake \
#           libopencv-dev libboost-all-dev libeigen3-dev \
#           libgomp1 libsuitesparse-dev git \
#           autoconf libtool"
#   end
#
#   namespace :trusty do
#     desc "Install GUI and non-GUI dependencies for Ubuntu trust"
#     task :gui => 'dependencies:trusty' do
#       sh "sudo apt-get install -y \
#         		libglew-dev libglm-dev freeglut3-dev"
#     end
#   end
#
#   desc "Install non-GUI depenencies on OSX using Brew"
#   task :osx_brew do
#     sh "brew update"
#     sh "brew tap homebrew/science"
#     sh "brew install homebrew/science/opencv homebrew/science/suitesparse \
#             eigen"
#   end
#
#   namespace :osx_brew do
#     desc "Install GUI and non-GUI dependencies on OSX using Brew"
#     task :gui => 'dependencies:osx_brew' do
#       sh "brew install glew glm" # freeglut"   Deprecate GLUT and use native windowing instead?
#     end
#   end
#
# end
