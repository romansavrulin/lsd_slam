
require 'pathname'

# Rake support files are stored in the .rake subdirectory
$:.unshift File.dirname(__FILE__) + "/.rake"
require 'docker'
require 'conan'
require 'dependencies'
require 'tests'
require 'build'
require 'build_tasks'

## Set defaults
@cmake_opts = ['-DBUILD_UNIT_TESTS:BOOL=True']
@conan_opts = {}
@conan_settings = {}
@conan_scopes = { build_tests: 'True' }

@coverity_email = ENV['LSDSLAM_COVERITY_EMAIL']
@coverity_token = ENV['LSDSLAM_COVERITY_TOKEN']

@build_parallelism = nil

## Any of the above configuration variables can be overridden in a config.rb
## file
load 'config.rb' if FileTest::exists? 'config.rb'


## Builds occur in directories "#{BUILD_ROOT}-#{build_type}"
## e.g. build-debug/, build-release/
build_root = ENV['BUILD_ROOT'] || "build"


newBuilds = [ Build.new( "Debug" ) ]
BuildTasks.new( newBuilds )


task :default => "debug:test"

builds = %w( Release Debug_NoGUI )
builds.each do |build|

  deps_touchfile = '.DEPS_MADE'

  namespace build.downcase do


    build_type = build.split('_').first
    cmake_args = %W( -DCMAKE_BUILD_TYPE:string=#{build_type}
                  #{ENV['CMAKE_FLAGS']}
                  #{@cmake_opts.join(' ')} )

    cmake_args << "-DEXTERNAL_PROJECT_PARALLELISM:string=#{@build_parallelism}" if @build_parallelism

    @cmake_opts << '-DBUILD_GUI:bool=False' if( build =~ /NoGUI/  )

    desc "Make lsd_slam for #{build}"
    task :build => ["cmake", "deps", "make"]

    build_dir = [build_root, build].join('-')

    directory build_dir do
      mkdir build_dir unless FileTest.directory? build_dir
    end

    task :cmake => build_dir do
      chdir build_dir do
        sh "cmake % s .." % cmake_args.join(' ')
      end
    end

    task :deps => "cmake" do
      chdir build_dir do
        sh "make deps && touch #{deps_touchfile}" unless File.exists? deps_touchfile
      end
    end

    def do_make( prefix = "" )
      if @build_parallelism and @build_parallelism > 0
        sh "#{prefix} make -j#{@build_parallelism}"
      else
        sh "#{prefix} make"
      end
    end

    task :make => "deps" do
      chdir build_dir do
        do_make
      end
    end


    task :coverity => "deps" do
      chdir build_dir do
        do_make "cov-build --dir cov-int"
        sh "tar -czvf ../coverity-#{build}-lsdslam.tar.gz cov-int"
      end
    end

    namespace :coverity do
      task :submit do
        if @coverity_email and @coverity_token
        sh "curl -o response.txt \
              --form token=#{@coverity_token} \
              --form email=#{@coverity_email} \
              --form file=@coverity-#{build}-lsdslam.tar.gz \
              --form version=\"latest\" \
              --form description=\"Git commit #{`git rev-parse HEAD`}\" \
              https://scan.coverity.com/builds?project=amarburg%2Flsd_slam"
        else
          puts "@coverity_email and @coverity_token not set, not submitting to Coverity"
        end
      end
    end

    ## Force make deps
    # desc "Force rebuild of the dependencies for #{build}"
    # task :deps  do
    #   mkdir build_dir unless FileTest.directory? build_dir
    #   chdir build_dir do
    #     sh "cmake", *cmake_args
    #     FileUtils.rm deps_touchfile
    #     sh "make deps && touch #{deps_touchfile}"
    #   end
    # end

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

# Conan builds default to no GUI, so Debug_GUI needs to be explicitly included
ConanTasks.new( builds: %w( Release Debug Debug_GUI ), opts: @conan_opts, settings: @conan_settings, scopes: @conan_scopes )

DockerTasks.new( builds: builds )
BenchmarkTasks.new( builds: builds )
