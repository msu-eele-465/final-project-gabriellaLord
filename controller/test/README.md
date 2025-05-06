# Tests

This folder contains all test-related files.

> [!TIP]
> To the extent possible, each module should be tested independently!

## Test organization

There are multiple ways to organize test files. For example, you may create subdirectories for each module and have one or more test files in the subdirectories; alternatively, you could put all test files in the top-level `test` folder.

**Subdirectories, using one test file per module**
- keypad
    - keypad_test.c
- rgb-led
    - rgb_test.c

**No subdirectories**
- keypad_test.c
- rgb_test.c

## Running tests

To be able to run the tests without using the main application code (`app/main.c`), our test code needs to have a `main()` function:
```c
#include <msp430fr2355.h>

int main(void)
{
    // Run test functions here...
}
```
You can choose to have a single `main()` function that is edited to run different tests, or you can have a separate `main()` function (in separate files) for each module or set of tests.

> [!IMPORTANT]
> Our CCS projects can't have more than one `main()` function; we have to exclude files from the CCS project so we only have one `main()` function visible at a time.
> Once the desired `main()` function that runs the tests is the only `main()` function visible to the CCS project, you can run your tests by building and running the project.

There are two primary ways to exclude files so we can run tests:
- Create separate build configurations for each set of tests (e.g., each module); within the build configuration, you can exclude all other files that contain a `main()` function except the test file you want to run.
- Use one build configuration for everything and modify which files are excluded every time you want to run something different (e.g., a different test or the actual main application).

### Excluding files

See the [Exclude Files from Build](https://software-dl.ti.com/ccs/esd/documents/users_guide_ccs_20.0.2/ccs_project-management.html#exclude-files-from-build) section of the CCS user guide to learn how to exclude files.


### Using separate build configurations

In this scenario, you will [create a build configuration](https://software-dl.ti.com/ccs/esd/documents/users_guide_ccs_20.0.2/ccs_project-management.html#build-configurations) for each set of tests you want to run (e.g., your keypad tests). In each build configuration, you will
- ensure your test file is included in the build configuration
- exclude the `app` folder from the project (right-click -> exclude from build)
- exclude any other test files that have a main function

When you want to run a different set of tests or the main application, you just need to switch your build configuration. Each build configuration will store settings for which files are included and excluded from the build.


### Using one build configuration

In this scenario, you will always use the `Debug` build configuration, which is the default configuration.

When you want to run a test, you will need to
- ensure your test file is included in the build configuration
- exclude the `app` folder from the project (right-click -> exclude from build)
- exclude any other test files that have a main function

When you want to run a different set of tests or the main application, you will need to update which files are included and excluded from the build.
