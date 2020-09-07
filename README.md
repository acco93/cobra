This repository contains a set of components for developing heuristic solution approaches to the Capacitated Vehicle Routing Problem (CVRP).
In particular, the library provides functionalities for managing instances and solutions, and a local search engine making use of several acceleration techniques such as Granular Neighborhoods and Static Move Descriptors.

An algorithm built with COBRA is available [here](https://github.com/acco93/filo).

### How to use the library
1. Clone the repository
    ```
    git clone https://github.com/acco93/cobra.git
    ```
2. Build the library
    ```
    cd cobra
    mkdir build && cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release
    make -j
    ```
3. Install it
    ```
    sudo make install
    ```
4. Add and link the library to your cmake project
    ```
    find_package(cobra 1.0.0 REQUIRED)
    add_executable(your-app-name main.cpp)
    target_link_libraries(your-app-name PUBLIC cobra)
    ```

##### Can I uninstall the library?
Sure, just go back to the build directory and execute
```
sudo make uninstall
```  
