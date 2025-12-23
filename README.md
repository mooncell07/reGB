<p align="center"> <img src="https://raw.githubusercontent.com/mooncell07/reGB/master/static/zelda.gif" alt="Zelda Gameplay" height=200></p>
<h1 align="center">re:GB</h1>

<h2 align="center">
<img alt="Stargazers" src="https://img.shields.io/github/stars/mooncell07/reGB?style=for-the-badge&logo=starship&color=C9CBFF&logoColor=D9E0EE&labelColor=302D41">
<img alt="Issues" src="https://img.shields.io/github/issues/mooncell07/reGB?style=for-the-badge&logo=gitbook&color=B5E8E0&logoColor=D9E0EE&labelColor=302D41">
<img alt="License" src="https://img.shields.io/github/license/mooncell07/reGB?style=for-the-badge&logo=github&color=F2CDCD&logoColor=D9E0EE&labelColor=302D41"/>
<img alt="Build Status" src="https://img.shields.io/github/actions/workflow/status/mooncell07/reGB/Linux-Workflow.yml?style=for-the-badge&logo=githubactions&logoColor=D9E0EE&labelColor=302D41&color=a6e3a1"/>
</h2>

re:GB is a hobby Nintendo Gameboy Emulator in C. Currently, the emulator lacks quite a few vital features including sound, state saving capabilities and also only MBC1
is supported as of now. I am working towards adding these features, especially audio. Anyways, to try it out follow the instructions below. Btw you would need to 
supply your own legally acquired ROM. *re:GB does NOT supply Proprietary ROMS and will never do it in future too.*

## Downloads
- [Linux Executable](https://nightly.link/mooncell07/reGB/workflows/Linux-Workflow/master/Linux%20Executable.zip)

## Usage

Clone the repo and build it yourself or download the precompiled binary.
Once you have the binary run:

```
reGB [filepath]
```

**NOTE**: You need a bootrom to run this emulator. Create a directory `roms` and put the `bootrom.gb` file there. If you don't have any then download the bootrom from this repo's `roms/` directory and put it in your local `roms/` directory.
Both binary and `roms/` (with `bootrom.gb` file inside it) should be in same directory.

## Keyboard Mapping

| Joypad Keys | Mapping     |
|-------------|-------------|
| A           | Z           |
| B           | X           |
| SELECT      | RIGHT SHIFT |
| START       | ENTER       |

*UP, DOWN, RIGHT & LEFT are mapped to corresponding ARROW KEYS on your keyboard*

## Tests & Game Screenshots

### Super Mario Land
<img src="https://raw.githubusercontent.com/mooncell07/reGB/master/static/super-mario.gif" alt="Demo of Super Mario Land" height=200>

### <a href="https://github.com/Hacktix/scribbltests/tree/master/fairylake">Fairylake</a>
<img src="https://raw.githubusercontent.com/mooncell07/reGB/master/static/fairylake.gif" alt="Fairylake Test Result" height=200>

### <a href="https://github.com/mattcurrie/dmg-acid2"> dmg-acid2</a>
<img src="https://raw.githubusercontent.com/mooncell07/reGB/master/static/dmg-acid2.png" alt="dmg-acid2 Test Result" height=200>

## Compilation/Development Instructions

You need to have `cmake` and `clang` in order to build the project.
You also need `python`, `poetry` and optionally `clang-format` for auto-generating the CPU Backend and/or testing the project with JSON Tests.

Firstly, clone this repo *along with* its submodules.
Then run the following command:
```
cd vcpkg && ./bootstrap-vcpkg.sh
```
Once that is done, run the commands:
```
cmake --preset [release | debug | test]
```
```
cmake --build --preset [release | debug | test]
```

- Use `release` to generate a release build with optimizations.
- Use `debug` to generate a debug build.
- Use `test` to generate a shared object, used for loading in a python script to run JSON Tests.

## Autogen Instructions

This Emulator uses auto-generated CPU Instruction Handlers. [See More](./codegen/README.md).
