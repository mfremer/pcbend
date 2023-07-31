#include "Settings.hpp"

#include <LibSL/LibSL.h>

#include <iostream>

int main(int argc, char* argv[]) {
    Settings stgs = read_settings("../data/test.cfg");
    dump_settings(stgs);
    return EXIT_SUCCESS;
}