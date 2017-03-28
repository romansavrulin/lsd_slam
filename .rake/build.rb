
require 'rake'


class Build

  include Rake::DSL

  attr_reader :name

  def initialize( name, opts = {} )
    @name = name

    @GUI = opts[:GUI] || true
  end

  def noGUI?
    !@GUI
  end

  def downcase
    name.downcase
  end

  def cmakeType
    name
  end

  def in_namespace( &blk )
    namespace downcase do
      blk.yield 
    end
  end

end
