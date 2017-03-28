require 'rake'
require 'pathname'

#
#
#
class DockerTasks

  include Rake::DSL

  def initialize( opts = {})

    @builds = opts[:builds] || %w( debug release )

    @root_dir = Pathname.new(__FILE__).parent.parent
    @docker_path = @root_dir.join('.docker')

    define_image_tasks
    define_build_tasks
  end


  def docker_image
    "lsdslam-build:local"
  end

  def coverity_image
    "lsdslam-coverity:local"
  end

  def docker_run( opts )

    image = if opts[:coverity]
              coverity_image
            else
              docker_image
            end

    docker_opts = %W( -v #{@root_dir}:/home/lsdslam/lsd_slam
                    --env BUILD_ROOT=build_docker_#{image.gsub(/:/,'_')}
                    #{image})

    sh "docker run #{docker_opts.join(' ')} #{opts[:cmd]}"
  end

  def in_docker(*paths)
    chdir @docker_path.join(*paths) do
      yield
    end
  end

  def define_image_tasks

    namespace :docker do

      task :image => "docker:image:build"

      desc "Build test docker image."
      namespace :image do

        task :build do
          in_docker {
            sh "docker build --pull -t #{docker_image} ."
          }
        end

        task :coverity do
          in_docker("with_coverity") {
            sh "docker build -t #{coverity_image} ."
          }
        end
      end
    end
  end

  def define_build_tasks

      namespace :docker do

      @builds.each do |build|
        build = build.downcase

        namespace build do

          %w( deps make build test clean ).each {|verb|
            desc "Make #{verb} for #{build} in Docker"
            task verb do
              docker_run( cmd: "rake #{build}:#{verb}")
            end
          }

          task :coverity do
            docker_run( coverity:true, cmd: "rake #{build}:coverity")
          end

        end
      end

      desc "Open console in Docker"
      task :console do
        in_docker {
          args = %w(docker run -ti --entrypoint /bin/bash) + docker_run_opts
          sh *args
        }
      end
    end

  end


end
