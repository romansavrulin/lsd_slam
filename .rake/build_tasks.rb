
require 'rake'
require 'cmake'

class BuildTasks
  include Rake::DSL

  def initialize( builds, opts = {} )
    @builds = builds
    @cmake = opts[:cmake] || CMake.new

    @build_root = "build"

    define_tasks
  end

  def define_tasks
    @builds.each { |build|
      build.in_namespace do
        define_build_tasks( build )
      end
    }
  end

  def define_build_tasks( build )

      deps_touchfile = '.DEPS_MADE'

      desc "Make lsd_slam for \"#{build.name}\""
      task :build => ["cmake", "deps", "make"]

      build_dir = [@build_root, build.name].join('-')

      directory build_dir do
        mkdir build_dir unless FileTest.directory? build_dir
      end

      task :cmake => build_dir do
        @cmake.define( :BUILD_UNIT_TESTS, :bool, "True" )
        @cmake.define( :BUILD_GUI, :bool, "False") if build.noGUI?
        @cmake.build_type( build.cmakeType )
        @cmake.define( :EXTERNAL_PROJECT_PARALLELISM, :string, @build_parallelism ) if @build_parallelism

        chdir build_dir do
          sh "cmake %s .." % @cmake.opts
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


    #
    #
    #
    #     task :coverity => "deps" do
    #       chdir build_dir do
    #         do_make "cov-build --dir cov-int"
    #         sh "tar -czvf ../coverity-#{build}-lsdslam.tar.gz cov-int"
    #       end
    #     end
    #
    #     namespace :coverity do
    #       task :submit do
    #         if @coverity_email and @coverity_token
    #         sh "curl -o response.txt \
    #               --form token=#{@coverity_token} \
    #               --form email=#{@coverity_email} \
    #               --form file=@coverity-#{build}-lsdslam.tar.gz \
    #               --form version=\"latest\" \
    #               --form description=\"Git commit #{`git rev-parse HEAD`}\" \
    #               https://scan.coverity.com/builds?project=amarburg%2Flsd_slam"
    #         else
    #           puts "@coverity_email and @coverity_token not set, not submitting to Coverity"
    #         end
    #       end
    #     end
    #
    #     ## Force make deps
    #     # desc "Force rebuild of the dependencies for #{build}"
    #     # task :deps  do
    #     #   mkdir build_dir unless FileTest.directory? build_dir
    #     #   chdir build_dir do
    #     #     sh "cmake", *cmake_args
    #     #     FileUtils.rm deps_touchfile
    #     #     sh "make deps && touch #{deps_touchfile}"
    #     #   end
    #     # end
    #

    #
    # end
    # )

end
