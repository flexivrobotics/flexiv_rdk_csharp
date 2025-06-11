# Flexiv RDK CSharp
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://www.apache.org/licenses/LICENSE-2.0.html)

**Flexiv RDK CSharp** provides C# developers with an easy way to work with [**Flexiv RDK**](https://github.com/flexivrobotics/flexiv_rdk). It uses [**P/Invoke**](https://learn.microsoft.com/en-us/dotnet/standard/native-interop/pinvoke) to call most of the functions from the RDK C++ interface, excluding *flexiv::rdk::Model* and *flexiv::rdk::Scheduler*.

## Reference

For detailed usage instructions and API documentation, refer to the [Flexiv RDK Home Page](https://www.flexiv.com/software/rdk).

## Compatibility

| **Language** | **Supported OS** | **Platform** | **Compiler / Runtime** | **Notes**                        |
| ------------ | ---------------- | ------------ | ---------------------- | -------------------------------- |
| **C++**      | Windows 10+      | x64          | MSVC v14.2+ (VS 2019+) | Builds dynamic library (`.dll`)  |
| **C#**       | Windows 10+      | x64          | .NET 5.0 or later      | Uses `DllImport` to call C++ DLL |

## Project Structure

This project is a **Windows 10** solution developed using **Visual Studio 2019**. It calls the C++ interfaces from [flexiv_rdk](https://github.com/flexivrobotics/flexiv_rdk) via P/Invoke and wraps them with C# interfaces.

- **FlexivRdkCSharp/**: C# project containing the wrapper and usage examples
  - **FlexivRdk/**: C# wrapper classes calling functions from the native DLL
  - **Examples/**: Example programs demonstrating usage of the C# wrappers
  - `Program.cs`: Entry point, runs selected example programs
- **FlexivRdkDll/**: C++ project wrapping **flexiv_rdk** C++ interfaces and generating a dynamic library (`.dll`)
- **ReleaseDll/**: Precompiled dynamic libraries for immediate use

## Usage

1. Use the `.cs` files from the **FlexivRdkCSharp/FlexivRdk/** folder in your project.
2. Place the dynamic libraries from **ReleaseDll/** in the same directory as your generated C# `.exe` file.
3. The **FlexivRdkCSharp** and **FlexivRdkDll** projects are pre-configured to output to the same directory, with **FlexivRdkCSharp** set as the startup project.
4. Build **FlexivRdkDll** to generate the required DLLs.
5. You may need to update the following in the **FlexivRdkDll** project settings according to your environment:
   - **C/C++ → Additional Include Directories**
   - **Linker → Additional Library Directories**
   - **Linker → Additional Dependencies**
6. Users can extend the C# interfaces as needed for additional functionality.

## License

This project is licensed under the [Apache License 2.0](https://www.apache.org/licenses/LICENSE-2.0).
