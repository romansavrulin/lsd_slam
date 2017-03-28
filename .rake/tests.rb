
require 'rake'

class BenchmarkTasks

  include Rake::DSL

  def initialize( opts )
    @builds = opts[:builds]
    @root_dir = Pathname.new(__FILE__).parent.parent
    @dataset_dir = @root_dir.parent.join('datasets/')


    define_tasks
  end

  def define_tasks
    @builds.each { |build|

      namespace build.downcase do

        ## Nasty, ugly, nasty
        bin_dir = @root_dir.join("build-#{build}")
        namespace :bench do
          bench_directory = @root_dir.join("bench_results")

          desc "Run the \"LSD_room\" test set with \"#{build}\""
          task :room do
            room_dir = bench_directory.join("room")
            mkdir_p room_dir unless FileTest.directory? room_dir

            data_dir = @dataset_dir.join("LSD_room")

            chdir room_dir do
              sh "#{bin_dir}/LSD -c #{data_dir}/cameraCalibration.cfg -f #{data_dir}/images/"
            end
          end

        end
      end
    }
  end

end
