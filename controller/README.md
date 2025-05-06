# Main controller

This folder is the CCS project for the controller.

> [!IMPORTANT]
> The `controller` folder is the *CCS project*. This differs from previous projects where the `app` folder was the CCS project.

## Project organization

### 📁 Top-level folders

- [`app`](app): The main application (e.g., `main.c`)
- [`src`](src): Source code for modules that are specific to the controller (e.g., `keypad.c` and `keypad.h`)
- [`test`](test): Source code for test functions. See the [`test`](test) folder for more details on ways to set up tests.

#### CCS project folders

- [`.settings`](.settings): Project-specific CCS settings, if any.

### 📄 Top-level files

- [`lnk_msp430fr2355.cmd`](lnk_msp430fr2355.cmd): MSP430FR2355 linker file

#### CCS project files

- [`.ccsproject`](.ccsproject): Stores project settings contributed by TI.
- [`.cproject`](.cproject): Stores project settings contributed by CDT (C/C++ Development Tooling).
- [`.project`](.project): Stores project settings contributed by Eclipse.

> [!NOTE]
> See the CCS user guide for more info on the [CCS project metadata files](https://software-dl.ti.com/ccs/esd/documents/users_guide_ccs_20.0.2/ccs_project-management.html#create-project).
