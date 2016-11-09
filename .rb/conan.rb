require 'rake'

#
# Tasks for creating a Docker instance for testing
#

class ConanTasks

  include Rake::DSL

  def initialize( opts = {})
    @builds = opts[:builds] || %w( debug release )
    @build_opts = opts[:build_opts]
    define_tasks
  end

  def define_tasks

    namespace :conan do

      @builds.each do |build_type|

        namespace build_type.downcase.to_sym do
          build_dir = ENV['BUILD_DIR'] || "build-conan-#{build_type}"

          task :build do
            FileUtils::mkdir build_dir unless FileTest::directory? build_dir
            opencv_opt = "-o opencv_dir=%s" % @build_opts[:opencv_dir] if @build_opts[:opencv_dir]
            sh "cd %s && conan install --scope build_tests=True -s build_type=%s %s .. --build=missing" % [build_dir, build_type.capitalize, opencv_opt]
            sh "cd %s && conan build .." % [build_dir]
          end

          task :test => :build do
            sh "cd %s && make unit_test" % build_dir
          end

        end

      end

      task :export do
        sh "conan export amarburg/testing"
      end
      
    end

  end
end
