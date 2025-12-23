# Codegen

A python code generation script for generating instruction handlers for my C based Gameboy Emulator's CPU.
Any change you need to make to the backend should take place either in the main script or inside the template.

## Usage Instructions

Firstly install `poetry` and `clang-format`.
Once done `cd` to the root of this project and run the `export.sh` script.

```
chmod +x export.sh
./export.sh src/CPU/backend.c
```
*Skip the chmod command if you have already given the permission.*

## Manual Exporting

If for whatever reason `export.sh` cant be used then, in root of the `codegen` project run the command

```
poetry run python codegen
```

This will generate the code and print it to the terminal. You can redirect it to a file as

```
poetry run python codegen > ../src/CPU/backend.c
```

After that, it is strongly recommended to run `clang-format` on `backend.c`. This is because the generator
generates terribly formatted code and using `clang-format` drastically fixes the formatting.

## Read This

The `export.sh` script OVERWRITES the file you pass to it so be careful while passing the filepath.
There is a security mechanism in place, the script checks for the `#define __AUTOGEN__` header before overwriting
so it won't overwrite if the header is missing. It is also NOT recommended to put the header anywhere other than
the file you want to be populated with auto-generated content.

*The script uses data from dmgops.json which is sourced from https://github.com/izik1/gbops for its code generation.*

