# Coding conventions

## General coding and formatting guidelines

- Minimum C++ compiler version to be used: C++11

- Header/cpp file naming conventions (camel-case or snake-case)
  - camel-case (Ex. PointcloudProjector.h) if the file defines some class
  - snake-case (Ex. pointcloud_projector.h) if the file does not define any class

- Space margin for access-specifiers in a class
  - 4-point spacing for access specifier
  - 8-point spacing for members (variable and functions)

```c++
class A
{
    public:
        A();

        virtual ~A();

        someFunc();

    protected:
        someOtherFunc();

    private:
        int var_;
        float var_2_;
}
```

- Use 4 spaces for indentation

- Use Javadoc style for Doxygen

- Header guard format 
  - Use Geosoft convention rule 40
  - e.g. `KELO_GEOMETRY_COMMON_LINE_SEGMENT_H`)

- Indentation of class contents
  - Use `BS_Allman` scheme as defined in
    [clang-format](https://clang.llvm.org/docs/ClangFormatStyleOptions.html)

- Class names: camel case
  - e.g.: `PointcloudProjector`, `Point2D`

- Function names (except constructor and destructors): camel case with first
  character not capitalised
  - e.g. `distTo`, `calcIntersectionPointWith`

- Variable names: snake case
  - protected and private member variables should have a trailing `_`
    - e.g.: `mat_`, `passthrough_min_z_`
