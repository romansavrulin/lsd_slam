
class CMakeDefine
  attr_reader :type, :value

  def initialize( type, value )
    @type = type
    @value = value
  end

  def to_s( name )
    "-D#{name}:#{type}=#{value}"
  end
end

class CMake

  def initialize
    @cmake_defines = {}
  end

  def define( name, type, value )
    @cmake_defines[name] = CMakeDefine.new(type,value)
  end

  def build_type( type )
    define( :CMAKE_BUILD_TYPE, :string, type)
  end

  def opts
    @cmake_defines.map {|name,define|
      define.to_s(name)
    }.join(' ')
  end


end
